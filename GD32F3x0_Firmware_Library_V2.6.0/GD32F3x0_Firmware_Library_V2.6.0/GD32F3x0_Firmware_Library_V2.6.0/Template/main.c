#include "main.h"
#include "communicate_protocol.h"
#include "gd32f3x0.h"
#include "math.h"
#include "stdio.h"
#include "string.h"
#include "systick.h"
#include "soft_i2c.h"
#include <stdint.h>

#define delay_ms(x) delay_1ms(x)

/************************ 宏定义区域 ************************/
// I2C设备7位地址
#define ICM42670_ADDR 0x68
#define QMC5883P_ADDR 0x2c

#define I2C_IMU I2C0       // ICM42670挂载I2C0 (PA0/PA1)
#define I2C_MAG I2C0       // 磁力计挂载I2C0
#define USART_RS485 USART1 // RS485上报串口
#define I2C_TIMEOUT 10000U // I2C通信超时计数

// Flash参数存储分区（GD32F330 Flash每页1024字节，使用末尾3页）
#define FLASH_PAGE_SIZE 1024U
#define FLASH_ZONE_A 0x0800FC00U    // 姿态基准备份A
#define FLASH_ZONE_B 0x0800FD00U    // 姿态基准备份B
#define FLASH_GYRO_BIAS 0x0800FE00U // 陀螺温度零偏存储区

// 采样与报警配置
#define DT 0.01f                    // 10ms采样周期 100Hz
#define ANGLE_ALARM_THRESHOLD 10.0f // 角度偏移报警阈值10°
#define ALARM_FILTER_CNT 200U       // 2s防抖帧数(200*10ms)
#define MAG_DISTURB_THRESH 120.0f   // 地磁突变判定阈值

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
/**
 * @brief ICM42670芯片初始化
 */
void icm42670_init(void) {
  uint8_t who_am_i;

  who_am_i = i2c_reg_read(I2C_IMU, ICM42670_ADDR, 0x75);

  if (who_am_i != 0x67) {
    return;
  }

  i2c_reg_write(I2C_IMU, ICM42670_ADDR, 0x1F, 0x00);
  delay_ms(100);

  i2c_reg_write(I2C_IMU, ICM42670_ADDR, 0x1F, 0x0F);
  delay_ms(30);

  i2c_reg_write(I2C_IMU, ICM42670_ADDR, 0x21, 0x68);
  i2c_reg_write(I2C_IMU, ICM42670_ADDR, 0x20, 0x68);
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
  icm_raw.ax = ax_raw / 16384.0f;
  icm_raw.ay = ay_raw / 16384.0f;
  icm_raw.az = az_raw / 16384.0f;

  int16_t gx_raw = (buf[6] << 8) | buf[7];
  int16_t gy_raw = (buf[8] << 8) | buf[9];
  int16_t gz_raw = (buf[10] << 8) | buf[11];
  icm_raw.gx = gx_raw / 131.072f;
  icm_raw.gy = gy_raw / 131.072f;
  icm_raw.gz = gz_raw / 131.072f;

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

  mag_raw.mx = mx_raw / 2048.0f;  /* 转换为高斯 */
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
  float p = pitch_deg * 0.0174533f * 0.5f;
  float r = roll_deg * 0.0174533f * 0.5f;
  float y = yaw_deg * 0.0174533f * 0.5f;

  float cp = cosf(p), sp = sinf(p);
  float cr = cosf(r), sr = sinf(r);
  float cy = cosf(y), sy = sinf(y);

  quat.w = cp * cr * cy + sp * sr * sy;
  quat.x = sp * cr * cy - cp * sr * sy;
  quat.y = cp * sr * cy + sp * cr * sy;
  quat.z = cp * cr * sy - sp * sr * cy;

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
  static uint8_t first_run = 1;
  static float gx_bias = 0.0f, gy_bias = 0.0f;
  static float gx_sum = 0.0f, gy_sum = 0.0f, gz_sum = 0.0f;
  static uint16_t init_cnt = 0;
  static float ix = 0.0f, iy = 0.0f, iz = 0.0f;

  if (first_run) {
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;
    init_cnt++;

    if (init_cnt >= 200) {
      gx_bias = gx_sum / 200.0f;
      gy_bias = gy_sum / 200.0f;
      gyro_bias.gz_bias = gz_sum / 200.0f;
      gyro_bias.temp_ref = temp;

      float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.3f;
      float accel_roll = atan2f(ay, az) * 57.3f;
      euler_to_quat(accel_pitch, accel_roll, 0.0f);

      first_run = 0;
    } else {
      float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.3f;
      float accel_roll = atan2f(ay, az) * 57.3f;
      att.pitch = accel_pitch;
      att.roll = accel_roll;
      att.yaw_now = 0.0f;
      return;
    }
  }

  float gx_comp = gx - gx_bias;
  float gy_comp = gy - gy_bias;
  float gz_comp = gz - gyro_bias.gz_bias;

  static uint16_t stable_cnt = 0;
  uint8_t is_stable = (fabsf(gx_comp) < 2.0f) && (fabsf(gy_comp) < 2.0f) &&
                      (fabsf(gz_comp) < 2.0f);

  if (is_stable) {
    stable_cnt++;
  } else {
    stable_cnt = 0;
    ix = 0.0f;
    iy = 0.0f;
    iz = 0.0f;
  }

  float q0 = quat.w, q1 = quat.x, q2 = quat.y, q3 = quat.z;

  float gx_rad = gx_comp * 0.0174533f;
  float gy_rad = gy_comp * 0.0174533f;
  float gz_rad = gz_comp * 0.0174533f;

  float half_dt = DT * 0.5f;

  float dq0 = (-q1 * gx_rad - q2 * gy_rad - q3 * gz_rad) * half_dt;
  float dq1 = (q0 * gx_rad + q2 * gz_rad - q3 * gy_rad) * half_dt;
  float dq2 = (q0 * gy_rad - q1 * gz_rad + q3 * gx_rad) * half_dt;
  float dq3 = (q0 * gz_rad + q1 * gy_rad - q2 * gx_rad) * half_dt;

  float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
  if (stable_cnt > 10 && acc_norm > 0.85f && acc_norm < 1.15f) {
    float ax_n = ax / acc_norm;
    float ay_n = ay / acc_norm;
    float az_n = az / acc_norm;

    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    float ex = ay_n * vz - az_n * vy;
    float ey = az_n * vx - ax_n * vz;
    float ez = ax_n * vy - ay_n * vx;

    const float Kp = 0.5f;
    const float Ki = 0.001f;

    ix += ex * Ki;
    iy += ey * Ki;
    iz += ez * Ki;

    dq1 += (ex * Kp + ix) * half_dt;
    dq2 += (ey * Kp + iy) * half_dt;
    dq3 += (ez * Kp + iz) * half_dt;
  }

  quat.w += dq0;
  quat.x += dq1;
  quat.y += dq2;
  quat.z += dq3;

  quat_normalize();

  if (stable_cnt > 100) {
    gx_bias = gx_bias * 0.999f + gx * 0.001f;
    gy_bias = gy_bias * 0.999f + gy * 0.001f;
    gyro_bias.gz_bias = gyro_bias.gz_bias * 0.999f + gz * 0.001f;
  }

  q0 = quat.w;
  q1 = quat.x;
  q2 = quat.y;
  q3 = quat.z;

  float vx = 2.0f * (q1 * q3 - q0 * q2);
  float vy = 2.0f * (q0 * q1 + q2 * q3);
  float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  att.pitch = -atan2f(-vx, sqrtf(vy * vy + vz * vz)) * 57.3f;
  att.roll = -atan2f(vy, vz) * 57.3f;

  att.yaw_now += gz_comp * DT;

  while (att.yaw_now > 180.0f)
    att.yaw_now -= 360.0f;
  while (att.yaw_now < -180.0f)
    att.yaw_now += 360.0f;
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

/************************ GD32 Flash读写（存储安装标定零点）
 * ************************/
/**
 * @brief Flash读取，直接映射地址读取
 */
void flash_read(uint32_t addr, void *data, uint16_t len) {
  memcpy(data, (void *)addr, len);
}

/**
 * @brief Flash页擦除+字写入
 */
void flash_write(uint32_t addr, void *data, uint16_t len) {
  uint32_t page_addr = addr - (addr % FLASH_PAGE_SIZE);
  uint32_t *src = (uint32_t *)data;
  uint16_t word_cnt = len / 4U;

  fmc_unlock();
  fmc_page_erase(page_addr);
  for (uint16_t i = 0; i < word_cnt; i++) {
    fmc_word_program(addr + i * 4, src[i]);
  }
  fmc_lock();
}

/**
 * @brief 现场安装标定函数，采集500帧取平均保存基准零点
 * 设备安装调平后调用一次，断电永久保存
 * @note fmc_page_erase()会擦除整页，若后续Flash写入其他数据需注意分区规划
 */
int save_install_zero_point(void) {
  float yaw_sum = 0.0f, pit_sum = 0.0f, rol_sum = 0.0f;
  // 连续采集5秒数据取平均，消除瞬时抖动
  for (uint16_t i = 0; i < 500; i++) {
    icm42670_get_raw_data();
    attitude_calc_6axis(-icm_raw.az, icm_raw.ay, icm_raw.ax, -icm_raw.gz,
                        icm_raw.gy, icm_raw.gx, icm_raw.temp);
    yaw_sum += att.yaw_now;
    pit_sum += att.pitch;
    rol_sum += att.roll;
    delay_ms(10);
  }
  // 保存安装基准零点
  att.yaw_base = yaw_sum / 500.0f;
  att.pitch_base = pit_sum / 500.0f;
  att.roll_base = rol_sum / 500.0f;
  gyro_bias.temp_ref = icm_raw.temp;

  // 三份数据在同页内，合并为一次擦写（避免后续 flash_write
  // 擦除整页破坏前面数据）
  fmc_unlock();
  fmc_page_erase(FLASH_ZONE_A);
  fmc_word_program(FLASH_ZONE_A, ((uint32_t *)&att)[0]);
  fmc_word_program(FLASH_ZONE_A + 4, ((uint32_t *)&att)[1]);
  fmc_word_program(FLASH_ZONE_A + 8, ((uint32_t *)&att)[2]);
  fmc_word_program(FLASH_ZONE_A + 12, ((uint32_t *)&att)[3]);
  fmc_word_program(FLASH_ZONE_A + 16, ((uint32_t *)&att)[4]);
  fmc_word_program(FLASH_ZONE_A + 20, ((uint32_t *)&att)[5]);
  fmc_word_program(FLASH_ZONE_B, ((uint32_t *)&att)[0]);
  fmc_word_program(FLASH_ZONE_B + 4, ((uint32_t *)&att)[1]);
  fmc_word_program(FLASH_ZONE_B + 8, ((uint32_t *)&att)[2]);
  fmc_word_program(FLASH_ZONE_B + 12, ((uint32_t *)&att)[3]);
  fmc_word_program(FLASH_ZONE_B + 16, ((uint32_t *)&att)[4]);
  fmc_word_program(FLASH_ZONE_B + 20, ((uint32_t *)&att)[5]);
  fmc_word_program(FLASH_GYRO_BIAS, ((uint32_t *)&gyro_bias)[0]);
  fmc_word_program(FLASH_GYRO_BIAS + 4, ((uint32_t *)&gyro_bias)[1]);
  fmc_lock();

  attitude_info_t att_bak_a, att_bak_b;
  gyro_bias_t gyro_bias_bak;
  flash_read(FLASH_ZONE_A, &att_bak_a, sizeof(attitude_info_t));
  flash_read(FLASH_ZONE_B, &att_bak_b, sizeof(attitude_info_t));
  flash_read(FLASH_GYRO_BIAS, &gyro_bias_bak, sizeof(gyro_bias_t));
  // 检查写入是否成功
  if (att_bak_a.yaw_base != att.yaw_base ||
      att_bak_b.yaw_base != att.yaw_base ||
      att_bak_a.pitch_base != att.pitch_base ||
      att_bak_b.pitch_base != att.pitch_base ||
      att_bak_a.roll_base != att.roll_base ||
      att_bak_b.roll_base != att.roll_base ||
      gyro_bias_bak.gz_bias != gyro_bias.gz_bias ||
      gyro_bias_bak.temp_ref != gyro_bias.temp_ref) {
    return -1;
  } else {
    return 1;
  }
}

/**
 * @brief 上电加载安装标定零点
 */
void load_install_zero_point(void) {
  flash_read(FLASH_ZONE_A, &att, sizeof(attitude_info_t));
  flash_read(FLASH_GYRO_BIAS, &gyro_bias, sizeof(gyro_bias_t));

  if (isnan(att.pitch_base) || isnan(att.yaw_now) || att.pitch_base > 1000.0f ||
      att.pitch_base < -1000.0f) {
    att.pitch = 0.0f;
    att.roll = 0.0f;
    att.pitch_base = 0.0f;
    att.roll_base = 0.0f;
    att.yaw_base = 0.0f;
    att.yaw_now = 0.0f;
    gyro_bias.gz_bias = 0.0f;
    gyro_bias.temp_ref = 25.0f;
    // printf("Flash data invalid, using default zero point!\n");
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
  // 检测前后地磁向量模长，参数值判断磁场是否被干扰
  mag_disturb_detect();

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

  uint8_t trigger_alarm = 0;
  // 任意轴偏移超过10°触发故障判定
  if (pit_offset >= ANGLE_ALARM_THRESHOLD ||
      rol_offset >= ANGLE_ALARM_THRESHOLD ||
      yaw_offset >= ANGLE_ALARM_THRESHOLD) {
    trigger_alarm = 1;
  }

  // 2s防抖滤波，瞬时震动不报警
  if (trigger_alarm == 1) {
    alarm_filter_cnt++;
    if (alarm_filter_cnt > ALARM_FILTER_CNT) {
      fault_type = 0;
      // X/Y轴倾斜故障标记
      if (pit_offset >= ANGLE_ALARM_THRESHOLD ||
          rol_offset >= ANGLE_ALARM_THRESHOLD)
        fault_type = 0x01;
      // Z轴绕灯杆旋转故障标记
      if (yaw_offset >= ANGLE_ALARM_THRESHOLD)
        fault_type = 0x02;
      // send_alarm_info(fault_type, att.pitch, att.roll, att.yaw_now);
      alarm_filter_cnt = ALARM_FILTER_CNT;
    }
  } else {
    alarm_filter_cnt = 0;
  }
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
  delay_1ms(5);                         // 等 USART 线稳定
  volatile uint8_t dummy;
  for (int i = 0; i < 16; i++) {
    dummy = USART_RDATA(USART0);        // 读走残留字节
  }
  (void)dummy;
  usart_flag_clear(USART0, USART_FLAG_IDLE);
  usart_interrupt_flag_clear(USART0, USART_INT_FLAG_IDLE);
  memset(usart0_rx_buffer, 0, USART0_RX_BUF_SIZE);  // 清空 DMA 缓冲区
  usart0_rx_len = 0;
  usart0_rx_flag = 0;
  dma_transfer_number_config(DMA_CH2, USART0_RX_BUF_SIZE);  // 复位 DMA 计数器
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

int main(void) {
  systick_config();
  com_usart_init();
  // printf("Hellow word!\n");
  // sprintf(transmitter_buffer, "HELLO_world!\n");
  // usart_interrupt_enable(USART0, USART_INT_TBE);
  // delay_1ms(10);

  // txcount = 0;
  // usart_interrupt_enable(USART0, USART_INT_TBE);
  // printf("hello_word");
  /* configure RCU */
  /* I2C 软模拟初始化（soft_i2c_init 内部已配置 GPIO 和时钟） */
  soft_i2c_init();
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
      if (debug_cnt % 10 == 0) {
        // proto_send(usart0_rx_buffer[2]);
        // printf("imu_tmp = %.4f\r\n", icm_raw.temp);
        printf("mag_norm=%.4f,mag_x=%.4f,mag_y=%.4f,mag_z=%.4f\n",
               mag_raw.mag_norm, mag_raw.mx, mag_raw.my, mag_raw.mz);
        // printf("P=%.4f,R=%.4f,Y=%.4f\r\n", att.pitch, att.roll, att.yaw_now);
        // printf("ax=%.4f,ay=%.4f,az=%.4f\r\ngx=%.4f,gy=%.4f,gz=%.4f\r\n",
        //        icm_raw.ax, icm_raw.ay, icm_raw.az, icm_raw.gx, icm_raw.gy,
        //        icm_raw.gz);
        // printf("gx=%.4f,gy=%.4f,gz=%.4f\r\n", icm_raw.gx, icm_raw.gy,
        //        icm_raw.gz);
      }
    }
    // 处理串口数据（空闲中断已计算 usart0_rx_len）
    if (usart0_rx_flag) {
      usart0_rx_flag = 0;
      // if (usart0_rx_len >= 3) {
      //   uint8_t calc_cs =
      //       calc_checksum(usart0_rx_buffer + 1, usart0_rx_len - 2);
      //   if (calc_cs != usart0_rx_buffer[usart0_rx_len - 1]) {
      //     /* 校验失败，丢弃此帧 */
      //   } else {
      //     // proto_send(usart0_rx_buffer[2]);
      //   }
      // }
    }
  }
}

// #include "gd32f3x0.h"
// #include "stdio.h"
// #include "systick.h"

// #define delay_ms(x) delay_1ms(x)

// void com_usart_init(void)
// {
//     rcu_periph_clock_enable(RCU_GPIOA);
//     rcu_periph_clock_enable(RCU_USART0);

//     gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_9);
//     gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_10);

//     gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
//     gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ,
//     GPIO_PIN_9);

//     gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
//     gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ,
//     GPIO_PIN_10);

//     usart_deinit(USART0);
//     usart_word_length_set(USART0, USART_WL_8BIT);
//     usart_stop_bit_set(USART0, USART_STB_1BIT);
//     usart_parity_config(USART0, USART_PM_NONE);
//     usart_baudrate_set(USART0, 115200U);
//     usart_receive_config(USART0, USART_RECEIVE_ENABLE);
//     usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
//     usart_enable(USART0);
// }

// void timer_config(void)
// {
//     timer_parameter_struct timer_initpara;

//     rcu_periph_clock_enable(RCU_TIMER1);
//     timer_deinit(TIMER1);

//     timer_struct_para_init(&timer_initpara);
//     timer_initpara.prescaler = 8400 - 1;
//     timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
//     timer_initpara.counterdirection = TIMER_COUNTER_UP;
//     timer_initpara.period = 10000 - 1;
//     timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
//     timer_initpara.repetitioncounter = 0;
//     timer_init(TIMER1, &timer_initpara);

//     timer_interrupt_enable(TIMER1, TIMER_INT_UP);
//     nvic_irq_enable(TIMER1_IRQn, 0, 1);
//     timer_enable(TIMER1);
// }

// volatile uint32_t timer_count = 0;

// void TIMER1_IRQHandler(void)
// {
//     if (timer_interrupt_flag_get(TIMER1, TIMER_INT_FLAG_UP) == SET)
//     {
//         timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_UP);
//         timer_count++;
//     }
// }

// void main(void)
// {
//     systick_config();

//     printf("Step 1: Initializing USART...\n");
//     com_usart_init();
//     printf("Step 2: USART init done!\n");

//     printf("Step 3: Initializing TIMER1...\n");
//     timer_config();
//     printf("Step 4: TIMER1 init done!\n");

//     printf("Step 5: Entering main loop...\n");
//     while (1)
//     {
//         if (timer_count >= 100)
//         {
//             timer_count = 0;
//             printf("Timer interrupt working! Count: %lu\n", timer_count);
//         }
//     }
// }
