#include "main.h"
#include "communicate_protocol.h"
#include "gd32f3x0.h"
#include "hard_i2c.h"
#include "math.h"
#include "ota_protocol.h"
#include "stdio.h"
#include "string.h"
#include "systick.h"
#include <stdint.h>

/* ========== 禁用半主机（semihosting）==========
   标准 C 库的 printf 首次调用会走 semihosting（执行 BKPT 0xAB 请求调试器服务），
   没有调试器连接时触发 HardFault（HFSR.DEBUGEVT=1）。
   __use_no_semihosting 让链接器改用不含半主机的库版本，并保留浮点 %f 支持。 */
#pragma import(__use_no_semihosting)
struct __FILE { int handle; };
FILE __stdout;
void _sys_exit(int x) { x = x; }
void _ttywrch(int ch) { (void)ch; }

/* ========== printf 重定向到 USART0 ==========
   禁用半主机后必须自行提供 fputc，否则链接器会用 C 库自带的半主机版 fputc，
   导致 L6915E：__use_no_semihosting 与半主机 fputc 冲突。 */
int fputc(int ch, FILE *f) {
    usart_data_transmit(USART0, (uint8_t)ch);
    while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));
    return ch;
}

#define delay_ms(x) delay_1ms(x)

/************************ 宏定义区域 ************************/
// I2C设备7位地址
#define ICM42670_ADDR 0x68
#define QMC5883P_ADDR 0x2c

#define I2C_IMU I2C1       // ICM42670挂载I2C1 (PA0/PA1)
#define I2C_MAG I2C1       // 磁力计挂载I2C1
#define USART_RS485 USART1 // RS485上报串口
#define I2C_TIMEOUT 10000U // I2C通信超时计数（与hard_i2c.h保持一致）

// 采样与报警配置
#define DT 0.01f                  // 10ms采样周期 100Hz
#define MAG_DISTURB_THRESH 120.0f // 地磁突变判定阈值

/************************ 全局数据结构体 ************************/

// 全局变量
icm_raw_data_t icm_raw;
mag_raw_data_t mag_raw;
attitude_info_t att;
gyro_bias_t gyro_bias;
quaternion_t quat;

uint8_t day_mode = 1;                 // 1=白天6轴模式 0=夜间9轴融合
uint8_t mag_disturb_flag = 0;         // 地磁受大车干扰标记
uint8_t fault_type = 0;               // 偏转报警类型标记
static uint16_t alarm_filter_cnt = 0; // 报警防抖计数器
uint32_t stable_cnt;
volatile uint8_t imu_loop_flag = 0; // 定时器中断标志

#define ARRAYNUM(arr_nanme) (uint32_t)(sizeof(arr_nanme) / sizeof(*(arr_nanme)))
#define TRANSMIT_SIZE (ARRAYNUM(transmitter_buffer) - 1)

uint8_t transmitter_buffer[128] = "\n\rUSART interrupt test\n\r";
uint8_t receiver_buffer[32];
uint8_t transfersize = TRANSMIT_SIZE;
uint8_t receivesize = 32;
__IO uint8_t txcount = 0;
__IO uint16_t rxcount = 0;

/************************ USART0 DMA + 空闲中断接收 ************************/
#define USART0_RX_BUF_SIZE 256U
uint8_t usart0_rx_buffer[USART0_RX_BUF_SIZE];
volatile uint16_t usart0_rx_len = 0;
volatile uint8_t usart0_rx_flag = 0;

/************************ ICM42670 驱动函数 ************************/
/* 软I2C助手函数前向声明（定义在文件后部，供icm42670_init提前调用） */
static void diag_scl(uint8_t level);
static void diag_sda(uint8_t level);
static uint8_t diag_sda_in(void);
static void diag_start(void);
static void diag_stop(void);
static uint8_t diag_send_byte(uint8_t byte);
// static uint8_t diag_read_byte(uint8_t ack_bit);  /* 诊断用，已注释 */
static void diag_reg_write(uint8_t dev, uint8_t reg, uint8_t data);
/**
 * @brief ICM42670芯片初始化
 * @note
 * 硬件I2C对0x75(WHO_AM_I)读会时钟拉伸挂起（软I2C诊断已确认器件正常，ID=0x67），
 *       故此处改用软I2C写配置寄存器，配完后恢复硬件I2C引脚供主循环读数据。
 */
void icm42670_init(void) {
  printf("ICM42670 init: via soft-I2C ...\r\n");

  /* 切换到软I2C（GPIO开漏）模式 */
  gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,
                GPIO_PIN_0 | GPIO_PIN_1);
  gpio_output_options_set(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,
                          GPIO_PIN_0 | GPIO_PIN_1);
  diag_scl(1);
  diag_sda(1);
  delay_1ms(2);

  diag_reg_write(0x68, 0x1F, 0x00);
  delay_ms(100);

  diag_reg_write(0x68, 0x1F, 0x0F);
  delay_ms(30);

  diag_reg_write(0x68, 0x21, 0x68);
  diag_reg_write(0x68, 0x20, 0x68);

  /* 恢复硬件I2C引脚(AF4)，供主循环/后续硬件I2C使用 */
  gpio_af_set(GPIOA, GPIO_AF_4, GPIO_PIN_0);
  gpio_af_set(GPIOA, GPIO_AF_4, GPIO_PIN_1);
  gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_0 | GPIO_PIN_1);
  gpio_output_options_set(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,
                          GPIO_PIN_0 | GPIO_PIN_1);

  printf("ICM42670 init done\r\n");
}

/**
 * @brief 读取IMU全部原始数据并转换物理量
 */
void icm42670_get_raw_data(void) {
  uint8_t buf[12] = {0};
  i2c_reg_read_multi(I2C_IMU, ICM42670_ADDR, 0x0B, buf, 12);

  int16_t ax_raw = (buf[0] << 8) | buf[1];
  int16_t ay_raw = (buf[2] << 8) | buf[3];
  int16_t az_raw = (buf[4] << 8) | buf[5];
  float ax_val = ax_raw / 16384.0f;
  float ay_val = ay_raw / 16384.0f;
  float az_val = az_raw / 16384.0f;

  if (fabsf(ax_val) > 20.0f || fabsf(ay_val) > 20.0f || fabsf(az_val) > 20.0f) {
    return;
  }

  int16_t gx_raw = (buf[6] << 8) | buf[7];
  int16_t gy_raw = (buf[8] << 8) | buf[9];
  int16_t gz_raw = (buf[10] << 8) | buf[11];
  float gx_val = gx_raw / 131.072f;
  float gy_val = gy_raw / 131.072f;
  float gz_val = gz_raw / 131.072f;

  if (fabsf(gx_val) > 2500.0f || fabsf(gy_val) > 2500.0f ||
      fabsf(gz_val) > 2500.0f) {
    return;
  }
  icm_raw.ax = ax_val;
  icm_raw.ay = ay_val;
  icm_raw.az = az_val;
  icm_raw.gx = gx_val;
  icm_raw.gy = gy_val;
  icm_raw.gz = gz_val;

  uint8_t temp_buf[2] = {0};
  i2c_reg_read_multi(I2C_IMU, ICM42670_ADDR, 0x09, temp_buf, 2);
  int16_t temp_raw = (temp_buf[0] << 8) | temp_buf[1];
  icm_raw.temp = temp_raw / 132.48f + 25.0f;
}

/************************ 磁力计驱动 ************************/
void qmc5883p_init(void) {
  /* Step 1: 软复位 Control(0x0B) bit7=1 */
  i2c_reg_write(I2C_MAG, QMC5883P_ADDR, 0x0B, 0x80);
  delay_ms(100);

  /* Step 2: 退出复位 */
  i2c_reg_write(I2C_MAG, QMC5883P_ADDR, 0x0B, 0x00);
  delay_ms(100);

  /* Step 3: 读 Chip ID 校验 (0x00 = 0x80) */
  uint8_t id = i2c_reg_read(I2C_MAG, QMC5883P_ADDR, 0x00);
  if (id != 0x80) {
    printf("QMC5883P not detected, ID=0x%02X\r\n", id);
  } else {
    printf("got id: 0x%02X\r\n", id);
  }

  /* Step 4: 配置 Control(0x0B) bits3-2=10(±8G), set_and_reset_on
     0x0C = 0b00001100 */
  i2c_reg_write(I2C_MAG, QMC5883P_ADDR, 0x0B, 0x0C);
  delay_ms(10);
  /* Step 5: 配置 Control Register 1 (0x0A): 连续模式 + 200Hz + OSR1=8 + OSR2=2
     0x4F = 0b01001111 */
  i2c_reg_write(I2C_MAG, QMC5883P_ADDR, 0x0A, 0x4F);
  delay_ms(10);
}

/**
 * @brief 读取磁力计原始数据（LSB first），计算地磁模长
 * @note  ±8G 量程下 1LSB = 1/2048 G
 * @note  寄存器 0x00=ChipID, 0x01~0x06=数据, 从 0x01 开始读
 */
void qmc5883p_get_raw_data(void) {
  uint8_t buf[6] = {0};
  i2c_reg_read_multi(I2C_MAG, QMC5883P_ADDR, 0x01, buf, 6);

  /* QMC5883P 小端序: X_L, X_H, Y_L, Y_H, Z_L, Z_H */
  int16_t mx_raw = (buf[1] << 8) | buf[0];
  int16_t my_raw = (buf[3] << 8) | buf[2];
  int16_t mz_raw = (buf[5] << 8) | buf[4];

  mag_raw.mx = mx_raw / 2048.0f; /* 转换为高斯 */
  mag_raw.my = my_raw / 2048.0f;
  mag_raw.mz = mz_raw / 2048.0f;
  mag_raw.mag_norm = sqrtf(mag_raw.mx * mag_raw.mx + mag_raw.my * mag_raw.my +
                           mag_raw.mz * mag_raw.mz);
}

/**
 * @brief 判断地磁是否被大车金属干扰
 */
void mag_disturb_detect(void) {
  static float last_norm = 0;
  float delta = fabsf(mag_raw.mag_norm - last_norm);
  last_norm = mag_raw.mag_norm;
  mag_disturb_flag = (delta > MAG_DISTURB_THRESH) ? 1 : 0;
}

/************************ 陀螺静态零偏自动校准 ************************/
/**
 * @brief 设备静止时自动微调Z轴陀螺零偏，抑制温度漂移
 * 仅保存至内存，不写入Flash，上电重置
 */

/************************ 四元数辅助函数 ************************/
static void quat_normalize(void) {
  float norm = sqrtf(quat.w * quat.w + quat.x * quat.x + quat.y * quat.y +
                     quat.z * quat.z);
  if (norm > 0.0001f) {
    quat.w /= norm;
    quat.x /= norm;
    quat.y /= norm;
    quat.z /= norm;
  }
}

static void euler_to_quat(float pitch_deg, float roll_deg, float yaw_deg) {
  float p = pitch_deg * 0.0174533f * 0.5f; /* θ/2 */
  float r = roll_deg * 0.0174533f * 0.5f;  /* φ/2 */
  float y = yaw_deg * 0.0174533f * 0.5f;   /* ψ/2 */

  float cp = cosf(p), sp = sinf(p);
  float cr = cosf(r), sr = sinf(r);
  float cy = cosf(y), sy = sinf(y);

  /* 标准ZYX四元数（q1=roll分量, q2=pitch分量），与 Mahony 修正和
     四元数→欧拉角提取(vx/vy/vz)的约定保持一致。
     原实现 q1/q2 反了，导致 Mahony 修正方向错误→姿态缓慢漂移→
     累积到总倾斜90°时 roll 提取进入奇异点，跳到±180°来回振荡。 */
  quat.w = cy * cp * cr + sy * sp * sr;
  quat.x = cy * cp * sr - sy * sp * cr;
  quat.y = cy * sp * cr + sy * cp * sr;
  quat.z = sy * cp * cr - cy * sp * sr;

  quat_normalize();
}

/************************ 姿态解算算法 ************************/
/**
 * @brief 姿态解算（四元数版）
 * 内部使用四元数进行姿态更新，避免万向节锁
 * 输出仍然是欧拉角（pitch, roll, yaw_now）
 */
void attitude_calc_6axis(float ax, float ay, float az, float gx, float gy,
                         float gz, float temp) {
  // ====== 阶段0：静态变量（跨调用保持状态） ======
  static uint8_t first_run = 1;                // 首次运行标志，触发初始对准
  static float gx_bias = 0.0f, gy_bias = 0.0f; // 陀螺零偏估计值（在线校准）
  static float gx_sum = 0.0f, gy_sum = 0.0f, gz_sum = 0.0f; // 初始对准累加器
  static uint16_t init_cnt = 0;                             // 初始对准采样计数
  static float ix = 0.0f, iy = 0.0f, iz = 0.0f; // Mahony PI 控制器积分项
  static int temp_stable_cnt = 0;               // 温度稳定计数器
  static float last_temp = 0.0f;                // 上次温度值
  static int temp_diff_flag = 0;                // 温度变化标志
  // ====== 阶段1：初始对准（前100次采样，约1秒） ======
  if (first_run) {
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;
    init_cnt++;

    if (init_cnt >= 500) {
      // 100次采样完成，求均值作为陀螺静态零偏
      gx_bias = gx_sum / 500.0f;
      gy_bias = gy_sum / 500.0f;
      gyro_bias.gz_bias = gz_sum / 500.0f;
      gyro_bias.temp_ref = temp;

      // 用加速度计计算初始姿态角，初始化四元数（yaw=0）
      float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.3f;
      float accel_roll = atan2f(ay, az) * 57.3f;
      euler_to_quat(accel_pitch, accel_roll, att.yaw_now);

      first_run = 0;
    } else {
      // 初始化未完成，仅输出加速度计姿态，不积分
      float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.3f;
      float accel_roll = atan2f(ay, az) * 57.3f;
      att.pitch = accel_pitch;
      att.roll = accel_roll;
      att.yaw_now = att.yaw_now;
      return;
    }
  }

  // ====== 阶段2：陀螺零偏补偿 ======
  float gx_comp = gx - gx_bias;
  float gy_comp = gy - gy_bias;
  float gz_comp = gz - gyro_bias.gz_bias;
  // ====== 阶段3：静止检测 ======
  static uint16_t stable_cnt = 0;                // 连续静止采样计数
  uint8_t is_stable = (fabsf(gx_comp) < 0.6f) && // 三轴角速度均 < 2°/s
                      (fabsf(gy_comp) < 0.6f) && // 判定为静止状态
                      (fabsf(gz_comp) < 0.6f);
  // printf("stable_cnt=%d, is_stable=%d, gx_comp=%.2f, gy_comp=%.2f,
  // gz_comp=%.2f\r\n", stable_cnt, is_stable, gx_comp, gy_comp, gz_comp);
  if (is_stable) {
    stable_cnt++;
  } else {
    stable_cnt = 0;
    // 运动时清零 Mahony 积分项，防止积分饱和
    ix = 0.0f;
    iy = 0.0f;
    iz = 0.0f;
  }

  // ====== 阶段4：陀螺积分 → 四元数增量（一阶龙格库塔法） ======
  float q0 = quat.w, q1 = quat.x, q2 = quat.y, q3 = quat.z;

  // ====== 阶段4.1：反演锁死检测与恢复 ======
  // 若估计重力 v 与实测重力 a 方向相反（点积<0），说明滤波器锁死在 180°
  // 倒立解（ 叉积误差在 180°
  // 处为0，拉不回来）。此时用加速度计重新初始化四元数，立即拉回正确姿态。
  // 仅当加速度模长≈1g 时判定，避免剧烈运动误触发。
  {
    float acc_norm_chk = sqrtf(ax * ax + ay * ay + az * az);
    if (acc_norm_chk > 0.9f && acc_norm_chk < 1.1f) {
      float ax_n = ax / acc_norm_chk, ay_n = ay / acc_norm_chk,
            az_n = az / acc_norm_chk;
      float vx_c = 2.0f * (q1 * q3 - q0 * q2);
      float vy_c = 2.0f * (q0 * q1 + q2 * q3);
      float vz_c = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
      if (vx_c * ax_n + vy_c * ay_n + vz_c * az_n < 0.0f) {
        /* 反演锁死：用加速度计重新初始化四元数，保持当前 yaw */
        float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.3f;
        float accel_roll = atan2f(ay, az) * 57.3f;
        euler_to_quat(accel_pitch, accel_roll, att.yaw_now);
        ix = 0.0f;
        iy = 0.0f;
        iz = 0.0f; /* 清积分项 */
        q0 = quat.w;
        q1 = quat.x;
        q2 = quat.y;
        q3 = quat.z; /* 用新四元数继续 */
      }
    }
  }

  float gx_rad = gx_comp * 0.0174533f; // °/s → rad/s
  float gy_rad = gy_comp * 0.0174533f;
  float gz_rad = gz_comp * 0.0174533f;

  float half_dt = DT * 0.5f; // 半采样周期

  // 四元数微分方程: dq/dt = 0.5 * q ⊗ ω
  float dq0 = (-q1 * gx_rad - q2 * gy_rad - q3 * gz_rad) * half_dt;
  float dq1 = (q0 * gx_rad + q2 * gz_rad - q3 * gy_rad) * half_dt;
  float dq2 = (q0 * gy_rad - q1 * gz_rad + q3 * gx_rad) * half_dt;
  float dq3 = (q0 * gz_rad + q1 * gy_rad - q2 * gx_rad) * half_dt;

  // ====== 阶段5：加速度计 Mahony 互补滤波修正（仅修正 pitch/roll） ======
  float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
  // 条件：连续静止 > 100ms 且加速度模长接近 1g（排除剧烈运动干扰）
  if (stable_cnt > 10 && acc_norm > 0.9f && acc_norm < 1.1f) {
    float ax_n = ax / acc_norm; // 归一化加速度
    float ay_n = ay / acc_norm;
    float az_n = az / acc_norm;

    // 从当前四元数提取重力方向在机体坐标系下的投影（理论值）
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    // 叉积求姿态误差：加速度实测值 × 四元数估计值
    float ex = ay_n * vz - az_n * vy;
    float ey = az_n * vx - ax_n * vz;
    float ez = ax_n * vy - ay_n * vx;

    // PI 控制器：比例项快速收敛 + 积分项消除稳态误差
    const float Kp = 20.0f; // 比例增益 0.5
    const float Ki = 0.001f;  // 积分增益 0.001
    ix += ex * Ki;
    iy += ey * Ki;
    iz += ez * Ki;

    // 积分限幅：防止积分项饱和后把姿态推跑（自激翻滚的诱因之一）
    const float I_LIMIT = 0.5f;
    if (ix > I_LIMIT)
      ix = I_LIMIT;
    else if (ix < -I_LIMIT)
      ix = -I_LIMIT;
    if (iy > I_LIMIT)
      iy = I_LIMIT;
    else if (iy < -I_LIMIT)
      iy = -I_LIMIT;
    if (iz > I_LIMIT)
      iz = I_LIMIT;
    else if (iz < -I_LIMIT)
      iz = -I_LIMIT;

    // printf(" ez = %.4f, iz = %.4f, yaw=%.5f, gz_bias=%.5f\r\n", ez, iz,
    // att.yaw_now, gyro_bias.gz_bias);

    // 将修正量叠加到四元数增量（d3 修正 pitch/roll，不修正 yaw）
    dq1 += (ex * Kp + ix) * half_dt;
    dq2 += (ey * Kp + iy) * half_dt;
    dq3 += (ez * Kp + iz) * half_dt;
  }

  // ====== 阶段6：更新四元数并归一化 ======
  quat.w += dq0;
  quat.x += dq1;
  quat.y += dq2;
  quat.z += dq3;

  quat_normalize();

  // ====== 阶段8：四元数 → 欧拉角输出 ======
  q0 = quat.w;
  q1 = quat.x;
  q2 = quat.y;
  q3 = quat.z;

  // 重力方向在机体坐标系下的投影
  float vx = 2.0f * (q1 * q3 - q0 * q2);
  float vy = 2.0f * (q0 * q1 + q2 * q3);
  float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  att.pitch = -atan2f(-vx, sqrtf(vy * vy + vz * vz)) * 57.3f;
  att.roll = -atan2f(vy, vz) * 57.3f;

  // ====== 阶段7：长时间静止时在线校准陀螺零偏 ======
  temp_stable_cnt++;
  if (temp_stable_cnt % 300 == 0) { // 每次加速纠正零偏持续30s
    if (fabsf(temp - last_temp) >= 0.5f) {
      last_temp = temp;
      temp_stable_cnt = 0;
      temp_diff_flag = 1;
    } else {
      temp_diff_flag = 0;
    }
  }
  if (stable_cnt > 500 || temp_diff_flag) {
    // 指数滑动平均跟踪零偏漂移，时间常数 τ ≈ 10s
    float stable_alpha = 0.01f; // 10s时间常数
    float fast_alpha = 0.1f;    // 温度漂移修正更快
    if (stable_cnt > 500) {
      gx_bias = gx_bias * (1 - stable_alpha) + gx * stable_alpha;
      gy_bias = gy_bias * (1 - stable_alpha) + gy * stable_alpha;
      gyro_bias.gz_bias = gyro_bias.gz_bias * (1 - stable_alpha) + gz * stable_alpha;
    } else if (stable_cnt > 500 && temp_diff_flag) {
      gx_bias = gx_bias * (1 - fast_alpha) + gx * fast_alpha;
      gy_bias = gy_bias * (1 - fast_alpha) + gy * fast_alpha;
      gyro_bias.gz_bias = gyro_bias.gz_bias * (1 - fast_alpha) + gz * fast_alpha;
    }
  } else {
    // 航向角纯陀螺积分（6轴模式无磁力计修正）
    att.yaw_now += gz_comp * DT;
  }
  // att.yaw_now += gz_comp * DT;
  // printf("is_stable=%d, yaw=%.5f, gz_bias=%.5f\r\n", is_stable, att.yaw_now,
  //        gyro_bias.gz_bias);
  // 角度归一化到 [-180°, 180°]
  while (att.yaw_now > 180.0f) {
    att.yaw_now -= 360.0f;
  }

  while (att.yaw_now < -180.0f) {
    att.yaw_now += 360.0f;
  }
}

/**
 * @brief 夜间9轴融合解算（无大车干扰，地磁缓慢修正Yaw积分漂移）
 */
void attitude_calc_9axis(void) {
  attitude_calc_6axis(-icm_raw.az, icm_raw.ay, icm_raw.ax, -icm_raw.gz,
                      icm_raw.gy, icm_raw.gx, icm_raw.temp);

  // 磁力计修正：仅在夜间模式、静止且无地磁干扰时执行
  if (day_mode == 0 && mag_disturb_flag == 0) {
    float pitch_rad = att.pitch * 0.0174533f;
    float cos_p = cosf(pitch_rad);

    if (fabsf(cos_p) > 0.15f) {
      float roll_rad = att.roll * 0.0174533f;
      float sin_p = sinf(pitch_rad);
      float cos_r = cosf(roll_rad);
      float sin_r = sinf(roll_rad);

      float mx_h = mag_raw.mx * cos_p + mag_raw.mz * sin_p;
      float my_h = mag_raw.mx * sin_r * sin_p + mag_raw.my * cos_r -
                   mag_raw.mz * sin_r * cos_p;

      float mag_yaw = atan2f(my_h, mx_h) * 57.3f;
      float yaw_err = mag_yaw - att.yaw_now;
      if (yaw_err > 180.0f)
        yaw_err -= 360.0f;
      if (yaw_err < -180.0f)
        yaw_err += 360.0f;

      att.yaw_now += yaw_err * 0.01f;
    }
  }
}

/************************ RS485串口上报函数 ************************/
/**
 * @brief 串口发送字符串
 */
void usart_send_string(char *str) {
  uint16_t len = strlen(str);
  for (uint16_t i = 0; i < len; i++) {
    while (usart_flag_get(USART_RS485, USART_FLAG_TBE) == RESET)
      ;
    usart_data_transmit(USART_RS485, str[i]);
  }
  while (usart_flag_get(USART_RS485, USART_FLAG_TC) == RESET)
    ;
}

/**
 * @brief 发送报警信息
 * @param fault_type:0x01倾斜故障 0x02旋转故障 0x03同时故障
 */
void send_alarm_info(uint8_t fault_type, float pit, float rol, float yaw) {
  static uint16_t alarm_print_cnt = 0;
  alarm_print_cnt++;
  if (alarm_print_cnt >= 100) {
    char buf[64] = {0};
    // sprintf(buf, "FAULT:%d,P=%.1f,R=%.1f,Y=%.1f\r\n", fault_type, pit, rol,
    //         yaw);
    // printf("%s", buf);
    alarm_print_cnt = 0;
  }
}

/************************ 昼夜模式切换（RTC小时判定） ************************/
void update_day_night_mode(uint8_t rtc_hour) {
  // 6点~22点白天6轴模式，其余夜间9轴融合
  if (rtc_hour >= 6 && rtc_hour < 22)
    day_mode = 1;
  else
    day_mode = 0;
}

/************************ 主循环调度函数（10ms定时调用）
 * ************************/
/**
 * @brief 定时10ms执行一次，传入RTC当前小时
 */
void imu_main_loop(uint8_t rtc_hour) {
  // 更新昼夜工作模式
  // printf("main loop\n");
  // usart_interrupt_enable(USART0, USART_INT_TBE);
  update_day_night_mode(rtc_hour);

  // 采集传感器原始数据
  icm42670_get_raw_data();
  qmc5883p_get_raw_data();
  static int16_t loop_cnt = 0;
  loop_cnt++;
  if (loop_cnt % 10 == 0) {
    // 检测前后地磁向量模长，参数值判断磁场是否被干扰
    mag_disturb_detect();
  } else {
    loop_cnt = 0;
  }

  // 分时姿态解算
  //   if (day_mode == 1)
  attitude_calc_6axis(-icm_raw.az, icm_raw.ay, icm_raw.ax, -icm_raw.gz,
                      icm_raw.gy, icm_raw.gx, icm_raw.temp);
  //   else
  //   attitude_calc_9axis();

  // 计算当前角度相对安装零点的偏移量
  float yaw_offset = fabsf(att.yaw_now - att.yaw_base);
  if (yaw_offset > 180.0f)
    yaw_offset = 360.0f - yaw_offset;
  float pit_offset = fabsf(att.pitch - att.pitch_base);
  float rol_offset = fabsf(att.roll - att.roll_base);

  uint8_t alarm_level = 0x00;
  // 检查严重偏转（超过严重阈值）
  if (pit_offset >= pitch_severe_threshold ||
      rol_offset >= roll_severe_threshold ||
      yaw_offset >= yaw_severe_threshold) {
    alarm_level = 0x02;
  }
  // 检查轻微偏转（超过轻微阈值，但未达严重）
  else if (pit_offset >= pitch_mild_threshold ||
           rol_offset >= roll_mild_threshold ||
           yaw_offset >= yaw_mild_threshold) {
    alarm_level = 0x01;
  }

  // 防抖滤波（预警时间可运行时修改）
  uint16_t alarm_time_max = alarm_warning_time * 100;
  if (alarm_level != 0x00) {
    alarm_filter_cnt++;
    if (alarm_filter_cnt > alarm_time_max) {
      fault_type = alarm_level;
      alarm_filter_cnt = alarm_time_max;
    }
  } else {
    alarm_filter_cnt = 0;
    fault_type = 0x00;
  }

  // 航向角断电保存（环形缓冲，内部自动判断写入时机）
  save_yaw_to_flash(att.yaw_now);
}

/************************ 系统总初始化入口 ************************/
/**
 * @brief 传感器系统初始化
 * 外部需提前初始化：GPIO、I2C0/I2C1、USART1、Systick、RTC、定时器
 */
void imu_system_init(void) {
  icm42670_init();
  qmc5883p_init();
  load_install_zero_point(); // 上电加载安装标定零点
  load_alarm_config();       // 上电加载报警参数
  load_yaw_from_flash();     // 上电恢复断电前航向角
}

// ## 外部补充初始化说明（需要自行添加）
// 1. `delay_ms()`：基于Systick实现毫秒延时函数；
// 2. I2C0、I2C1 GPIO+外设初始化；
// 3. USART1串口初始化（RS485收发）；
// 4. 定时器10ms中断，循环调用 `imu_main_loop(rtc_hour)`；
// 5. RTC时钟读取小时，传入主循环；
// 6. 上位机下发标定指令时调用 `save_install_zero_point()` 保存安装零点。

// ## 核心业务逻辑亮点
// 1.
// **大车磁场干扰规避**：白天车流高峰自动关闭地磁航向融合，仅用陀螺积分计算绕杆旋转，不会因货车金属车体造成角度跳变误报警；
// 2.
// **三轴统一报警**：俯仰X、横滚Y、旋转Z任意一轴偏移＞10°，持续2秒稳定后上报，区分倾斜/旋转两种故障码；
// 3. **断电保存标定零点**：Flash双备份存储安装基准，设备拆装后无需重新标定；
// 4. **陀螺温漂抑制**：设备静止时自动平滑更新Z轴零偏，长期运行角度漂移小；
// 5.
// **夜间漂移修正**：夜间车流稀少，地磁缓慢修正陀螺长时间积分误差，保证长期精度。

void com_usart_init(void) {
  // /* enable GPIO clock */
  // rcu_periph_clock_enable(RCU_GPIOA);
  // /* enable USART clock */
  // rcu_periph_clock_enable(RCU_USART0);

  // /* connect port to USART TX */
  // gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_9);

  // /* connect port to USART RX */
  // gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_10);

  // /* configure USART TX as alternate function push-pull */
  // gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
  // gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ,
  // GPIO_PIN_9);

  // /* configure USART RX as alternate function push-pull */
  // gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
  // gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ,
  // GPIO_PIN_10);
  // /* USART configure */
  // usart_deinit(USART0);
  // usart_word_length_set(USART0, USART_WL_8BIT);
  // usart_stop_bit_set(USART0, USART_STB_1BIT);
  // usart_parity_config(USART0, USART_PM_NONE);
  // usart_baudrate_set(USART0, 115200U);
  // usart_receive_config(USART0, USART_RECEIVE_ENABLE);
  // usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);

  // usart_enable(USART0);

  /* enable GPIO clock */
  rcu_periph_clock_enable(RCU_GPIOA);
  /* enable USART clock */
  rcu_periph_clock_enable(RCU_USART0);

  /* connect port to USART TX */
  gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_9);

  /* connect port to USART RX */
  gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_10);

  /* configure USART TX as alternate function push-pull */
  gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
  gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_9);

  /* configure USART RX as alternate function push-pull */
  gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
  gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_10);

  /* configure USART */
  usart_deinit(USART0);
  usart_word_length_set(USART0, USART_WL_8BIT);
  usart_stop_bit_set(USART0, USART_STB_1BIT);
  usart_parity_config(USART0, USART_PM_NONE);
  usart_baudrate_set(USART0, 9600U);
  usart_receive_config(USART0, USART_RECEIVE_ENABLE);
  usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
  usart_enable(USART0);
}

/************************ USART0 DMA + 空闲中断接收配置
 * ************************/
/**
 * @brief 配置USART0 DMA接收（循环模式）+ 空闲中断
 * @note  printf用的USART0，RX用DMA_CH2，空闲中断触发后算长度
 */
void usart0_rx_dma_idle_init(void) {
  dma_parameter_struct dma_para;

  rcu_periph_clock_enable(RCU_DMA);

  dma_deinit(DMA_CH2);
  dma_struct_para_init(&dma_para);

  dma_para.direction = DMA_PERIPHERAL_TO_MEMORY;
  dma_para.memory_addr = (uint32_t)usart0_rx_buffer;
  dma_para.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
  dma_para.memory_width = DMA_MEMORY_WIDTH_8BIT;
  dma_para.number = USART0_RX_BUF_SIZE;
  dma_para.periph_addr = (uint32_t)&USART_RDATA(USART0);
  dma_para.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
  dma_para.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
  dma_para.priority = DMA_PRIORITY_HIGH;
  dma_init(DMA_CH2, &dma_para);

  dma_circulation_disable(DMA_CH2); /* 单次模式，每帧重新配置 */
  dma_memory_to_memory_disable(DMA_CH2);

  usart_flag_clear(USART0, USART_FLAG_IDLE);
  usart_interrupt_flag_clear(USART0, USART_INT_FLAG_IDLE);

  usart_dma_receive_config(USART0, USART_RECEIVE_DMA_ENABLE);
  usart_interrupt_enable(USART0, USART_INT_IDLE);

  dma_channel_enable(DMA_CH2);
  nvic_irq_enable(USART0_IRQn, 1, 0);

  /* 清除开机时 USART 可能已收到的脏数据 */
  delay_1ms(5); // 等 USART 线稳定
  volatile uint8_t dummy;
  for (int i = 0; i < 16; i++) {
    dummy = USART_RDATA(USART0); // 读走残留字节
  }
  (void)dummy;
  usart_flag_clear(USART0, USART_FLAG_IDLE);
  usart_interrupt_flag_clear(USART0, USART_INT_FLAG_IDLE);
  memset(usart0_rx_buffer, 0, USART0_RX_BUF_SIZE); // 清空 DMA 缓冲区
  usart0_rx_len = 0;
  usart0_rx_flag = 0;
  dma_transfer_number_config(DMA_CH2, USART0_RX_BUF_SIZE); // 复位 DMA 计数器
}

/************************ USART0 DMA 发送 ************************/
volatile uint8_t usart0_tx_busy = 0;

/**
 * @brief USART0 DMA发送（USART0_TX = DMA_CH1）
 * @param buf: 待发送数据
 * @param len: 长度
 * @note  单次发送，非循环。发送期间 usart0_tx_busy = 1
 */
void usart0_tx_dma_send(uint8_t *buf, uint16_t len) {
  dma_parameter_struct dma_para;

  while (usart0_tx_busy)
    ; /* 等上一次发完 */

  rcu_periph_clock_enable(RCU_DMA);

  dma_deinit(DMA_CH1);
  dma_struct_para_init(&dma_para);

  dma_para.direction = DMA_MEMORY_TO_PERIPHERAL;
  dma_para.memory_addr = (uint32_t)buf;
  dma_para.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
  dma_para.memory_width = DMA_MEMORY_WIDTH_8BIT;
  dma_para.number = len;
  dma_para.periph_addr = (uint32_t)&USART_TDATA(USART0);
  dma_para.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
  dma_para.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
  dma_para.priority = DMA_PRIORITY_HIGH;
  dma_init(DMA_CH1, &dma_para);

  dma_circulation_disable(DMA_CH1);
  dma_memory_to_memory_disable(DMA_CH1);

  usart0_tx_busy = 1;

  /* 使能 DMA 传输完成中断，用于清除 busy 标志 */
  dma_interrupt_enable(DMA_CH1, DMA_INT_FTF);
  nvic_irq_enable(DMA_Channel1_2_IRQn, 0, 1);

  usart_dma_transmit_config(USART0, USART_TRANSMIT_DMA_ENABLE);
  dma_channel_enable(DMA_CH1);
}

/**
 * @brief DMA 发送完成中断（DMA_CH1 与 DMA_CH2 共用）
 * 清除 usart0_tx_busy，允许下一帧发送
 */
void DMA_Channel1_2_IRQHandler(void) {
  if (dma_interrupt_flag_get(DMA_CH1, DMA_INT_FLAG_FTF) == SET) {
    dma_interrupt_flag_clear(DMA_CH1, DMA_INT_FLAG_FTF);
    dma_channel_disable(DMA_CH1);
    usart_dma_transmit_config(USART0, USART_TRANSMIT_DMA_DISABLE);
    usart0_tx_busy = 0;
  }

  if (dma_interrupt_flag_get(DMA_CH2, DMA_INT_FLAG_FTF) == SET) {
    dma_interrupt_flag_clear(DMA_CH2, DMA_INT_FLAG_FTF);
  }
}

void timer_config(void) {
  timer_parameter_struct timer_initpara;

  /* 1. 使能 TIMER1 时钟 */
  rcu_periph_clock_enable(RCU_TIMER1);

  /* 2. 复位 TIMER1 */
  timer_deinit(TIMER1);

  /* 3. 配置 TIMER1 时基单元 */
  timer_struct_para_init(&timer_initpara);
  timer_initpara.prescaler =
      8400 - 1; // 预分频值 (系统时钟84MHz，分频后为10kHz)
  timer_initpara.alignedmode = TIMER_COUNTER_EDGE;    // 边缘对齐模式
  timer_initpara.counterdirection = TIMER_COUNTER_UP; // 向上计数模式
  timer_initpara.period = 100 - 1; // 自动重装载值 (100个计数周期 = 10ms)
  timer_initpara.clockdivision = TIMER_CKDIV_DIV1; // 时钟分频
  timer_initpara.repetitioncounter = 0;            // 重复计数器设为0
  timer_init(TIMER1, &timer_initpara);

  /* 4. 使能 TIMER1 更新中断 */
  timer_interrupt_enable(TIMER1, TIMER_INT_UP);

  /* 5. 配置 NVIC 中断优先级并使能中断 */
  nvic_irq_enable(TIMER1_IRQn, 0, 1);

  /* 6. 启动定时器 */
  timer_enable(TIMER1);
}

/************************ 软I2C诊断（验证0x68器件用，确认后可删除）
 * ************************/
static void diag_scl(uint8_t level) {
  if (level)
    gpio_bit_set(GPIOA, GPIO_PIN_0);
  else
    gpio_bit_reset(GPIOA, GPIO_PIN_0);
}
static void diag_sda(uint8_t level) {
  if (level)
    gpio_bit_set(GPIOA, GPIO_PIN_1);
  else
    gpio_bit_reset(GPIOA, GPIO_PIN_1);
}
static uint8_t diag_sda_in(void) {
  return gpio_input_bit_get(GPIOA, GPIO_PIN_1);
}
static void diag_start(void) {
  diag_sda(1);
  diag_scl(1);
  delay_us(5);
  diag_sda(0);
  delay_us(5);
  diag_scl(0);
  delay_us(5);
}
static void diag_stop(void) {
  diag_sda(0);
  diag_scl(1);
  delay_us(5);
  diag_sda(1);
  delay_us(5);
}
static uint8_t diag_send_byte(uint8_t byte) {
  uint8_t i, ack;
  for (i = 0; i < 8; i++) {
    diag_scl(0);
    delay_us(3);
    if (byte & 0x80)
      diag_sda(1);
    else
      diag_sda(0);
    delay_us(2);
    diag_scl(1);
    delay_us(5);
    byte <<= 1;
  }
  diag_scl(0);
  delay_us(3);
  diag_sda(1);
  delay_us(2);
  diag_scl(1);
  delay_us(5);
  ack = diag_sda_in(); /* 0=器件拉低SDA=ACK, 1=NACK */
  diag_scl(0);
  delay_us(5);
  return (ack == 0) ? 0 : 1;
}
/* 诊断用读字节函数，已注释
static uint8_t diag_read_byte(uint8_t ack_bit) {
  uint8_t i, byte = 0;
  diag_sda(1);
  for (i = 0; i < 8; i++) {
    diag_scl(0); delay_us(5);
    diag_scl(1); delay_us(3);
    byte <<= 1;
    if (diag_sda_in()) byte |= 0x01;
    delay_us(2);
  }
  diag_scl(0); delay_us(3);
  if (ack_bit) diag_sda(0); else diag_sda(1);
  delay_us(2); diag_scl(1); delay_us(5);
  diag_scl(0); delay_us(5);
  return byte;
}
*/
static void diag_reg_write(uint8_t dev, uint8_t reg, uint8_t data) {
  diag_start();
  diag_send_byte(dev << 1);
  diag_send_byte(reg);
  diag_send_byte(data);
  diag_stop();
}
/* 开机软I2C读WHO_AM_I诊断（调试用，已注释，正式运行不需要） */
// static void soft_i2c_whoami_diag(void) {
//   uint8_t ack_w, ack_reg, ack_r, who;
//
//   printf("\r\n[DIAG] Soft-I2C read WHO_AM_I: dev=0x68 reg=0x75 ...\r\n");
//
//   gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_0 |
//   GPIO_PIN_1); gpio_output_options_set(GPIOA, GPIO_OTYPE_OD,
//   GPIO_OSPEED_50MHZ, GPIO_PIN_0 | GPIO_PIN_1); diag_scl(1); diag_sda(1);
//   delay_1ms(2);
//
//   diag_start();
//   ack_w   = diag_send_byte(0x68 << 1);       /* 写地址 0xD0 */
//   ack_reg = diag_send_byte(0x75);            /* 寄存器 0x75 */
//   diag_start();                              /* RESTART */
//   ack_r   = diag_send_byte((0x68 << 1) | 1); /* 读地址 0xD1 */
//   who = diag_read_byte(0);                   /* 读1字节 + NACK */
//   diag_stop();
//
//   printf("[DIAG] ACK: addrW=%s reg=%s addrR=%s | WHO_AM_I=0x%02X %s\r\n",
//          ack_w ? "NACK" : "ACK", ack_reg ? "NACK" : "ACK", ack_r ? "NACK" :
//          "ACK", who, (who == 0x67) ? "(OK 0x67)" : "(NOT 0x67!)");
// }

int main(void) {
  /* App 链接在 0x08002000（bootloader 之后），启动必须把中断向量表重定位到 App 区 */
  SCB->VTOR = 0x08002000U;

  /* bootloader 跳转前调用了 __disable_irq() 关掉全局中断，必须恢复，
     否则 SysTick 中断不触发 → delay_1ms()/delay_ms() 会死循环卡死 */
  __enable_irq();

  systick_config();
  com_usart_init();
  printf("[APP] started\r\n");


  // soft_i2c_whoami_diag();   /*
  // 开机软I2C读WHO_AM_I诊断：调试用，正式运行注释掉 */ printf("Hellow
  // word!\n"); sprintf(transmitter_buffer, "HELLO_world!\n");
  // usart_interrupt_enable(USART0, USART_INT_TBE);
  // delay_1ms(10);

  // txcount = 0;
  // usart_interrupt_enable(USART0, USART_INT_TBE);
  // printf("hello_word");
  /* configure RCU */
  /* I2C1 硬件初始化（hard_i2c_init 内部已配置 GPIO 和 I2C1 外设） */
  hard_i2c_init();

  /* I2C总线扫描 - 检测设备是否存在（调试用，正式运行注释掉）
  printf("=== I2C Bus Scan ===\n");
  for(uint8_t addr = 0x08; addr <= 0x77; addr++) {
    if(hard_i2c_probe(addr) == 0) {
      printf("  Device found at 7-bit addr: 0x%02X (8-bit: 0x%02X)\n", addr,
  addr << 1);
    }
  }
  printf("=== Scan Done ===\n");
  */
  // printf("I2C init done!\n");

  // i2c_test();

  // // soft_i2c_test();

  // gpio_config();
  // i2c_config();

  // i2c_test();

  imu_system_init();
  usart0_rx_dma_idle_init(); /* 配置USART0 DMA接收 + 空闲中断 */
  timer_config();
  // printf("timer init done!\n");

  static uint32_t debug_cnt = 0;
  while (1) {
    if (imu_loop_flag) {
      imu_main_loop(12);
      imu_loop_flag = 0;

      debug_cnt++;
      /* 0x82 与 0x8E 错开 500ms 发送：
         proto_send 对 usart0_tx_busy 是"直接丢弃"而非等待，
         背靠背连发会导致第二帧被丢弃 */
      if (debug_cnt % 1000 == 0) {
        proto_send(0x82);
      } else if (debug_cnt % 1000 == 500) {
        proto_send(0x8E);
      }
        // printf("imu_tmp = %.4f\r\n", icm_raw.temp);
        // printf("mag_norm=%.4f,mag_x=%.4f,mag_y=%.4f,mag_z=%.4f\n",
        //        mag_raw.mag_norm, mag_raw.mx, mag_raw.my, mag_raw.mz);
        // printf("P=%.4f,R=%.4f,Y=%.4f\r\n", att.pitch, att.roll, att.yaw_now);
        // printf("ax=%.4f,ay=%.4f,az=%.4f\r\ngx=%.4f,gy=%.4f,gz=%.4f\r\n",
        //        icm_raw.ax, icm_raw.ay, icm_raw.az, icm_raw.gx, icm_raw.gy,
        //        icm_raw.gz);
        // printf("gx=%.4f,gy=%.4f,gz=%.4f\r\n", icm_raw.gx, icm_raw.gy,
        //        icm_raw.gz);
      }
    
    // 处理串口数据（空闲中断已计算 usart0_rx_len）
    if (usart0_rx_flag) {
      usart0_rx_flag = 0;
      if (usart0_rx_len >= 3) {
        uint8_t rx_cmd = usart0_rx_buffer[2];
        if (rx_cmd >= 0xF0) {
          /* OTA 远程烧录协议 */
          for (uint16_t i = 0; i < usart0_rx_len; i++) {
            ota_protocol_parse(usart0_rx_buffer[i]);
          }
        } else {
          /* 原有业务协议 */
          uint8_t calc_cs =
              calc_checksum(usart0_rx_buffer + 1, usart0_rx_len - 2);
          if (calc_cs != usart0_rx_buffer[usart0_rx_len - 1]) {
            /* 校验失败，丢弃此帧 */
          } else {
            proto_send(usart0_rx_buffer[2]);
          }
        }
      }
    }
  }
}