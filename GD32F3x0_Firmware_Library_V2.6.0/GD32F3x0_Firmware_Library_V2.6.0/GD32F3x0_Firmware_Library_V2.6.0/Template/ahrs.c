#include "ahrs.h"
#include "gd32f3x0.h"
#include "systick.h"
#include "soft_i2c.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

#define delay_ms(x) delay_1ms(x)

/* ========== I2C 设备地址 ========== */
#define ICM42670_ADDR  0x68
#define QMC5883P_ADDR  0x2C
#define I2C_IMU        I2C0
#define I2C_MAG        I2C0

/* ========== Flash 分区 ========== */
#define FLASH_PAGE_SIZE  1024U
#define FLASH_ZONE_A     0x0800FC00U
#define FLASH_ZONE_B     0x0800FD00U
#define FLASH_GYRO_BIAS  0x0800FE00U

/* ========== 内部辅助 ========== */

static void quat_normalize(quaternion_t *q) {
  float norm = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
  if (norm > 0.0001f) {
    q->w /= norm; q->x /= norm; q->y /= norm; q->z /= norm;
  }
}

static void euler_to_quat(quaternion_t *q, float pitch_deg, float roll_deg, float yaw_deg) {
  float p = pitch_deg * 0.0174533f * 0.5f;
  float r = roll_deg   * 0.0174533f * 0.5f;
  float y = yaw_deg    * 0.0174533f * 0.5f;

  float cp = cosf(p), sp = sinf(p);
  float cr = cosf(r), sr = sinf(r);
  float cy = cosf(y), sy = sinf(y);

  q->w = cp * cr * cy + sp * sr * sy;
  q->x = sp * cr * cy - cp * sr * sy;
  q->y = cp * sr * cy + sp * cr * sy;
  q->z = cp * cr * sy - sp * sr * cy;

  quat_normalize(q);
}

static void flash_read(uint32_t addr, void *data, uint16_t len) {
  memcpy(data, (void *)addr, len);
}

/* ========== 初始化 ========== */

void ahrs_init(ahrs_t *self) {
  memset(self, 0, sizeof(ahrs_t));
  self->first_run           = 1;
  self->day_mode            = 1;
  self->dt                  = 0.01f;
  self->angle_alarm_threshold = 10.0f;
  self->alarm_filter_max    = 200;
  self->mag_disturb_thresh  = 120.0f;
  self->gyro_bias.temp_ref  = 25.0f;
}

/* ========== ICM42670 ========== */

void ahrs_icm_read(ahrs_t *self) {
  uint8_t buf[12] = {0};
  i2c_reg_read_multi(I2C_IMU, ICM42670_ADDR, 0x0B, buf, 12);

  int16_t ax_raw = (buf[0] << 8) | buf[1];
  int16_t ay_raw = (buf[2] << 8) | buf[3];
  int16_t az_raw = (buf[4] << 8) | buf[5];
  self->icm.ax = ax_raw / 16384.0f;
  self->icm.ay = ay_raw / 16384.0f;
  self->icm.az = az_raw / 16384.0f;

  int16_t gx_raw = (buf[6] << 8) | buf[7];
  int16_t gy_raw = (buf[8] << 8) | buf[9];
  int16_t gz_raw = (buf[10] << 8) | buf[11];
  self->icm.gx = gx_raw / 131.072f;
  self->icm.gy = gy_raw / 131.072f;
  self->icm.gz = gz_raw / 131.072f;

  uint8_t temp_buf[2] = {0};
  i2c_reg_read_multi(I2C_IMU, ICM42670_ADDR, 0x09, temp_buf, 2);
  int16_t temp_raw = (temp_buf[0] << 8) | temp_buf[1];
  self->icm.temp = temp_raw / 132.48f + 25.0f;
}

static void icm42670_init_hw(void) {
  uint8_t who_am_i = i2c_reg_read(I2C_IMU, ICM42670_ADDR, 0x75);
  if (who_am_i != 0x67) return;

  i2c_reg_write(I2C_IMU, ICM42670_ADDR, 0x1F, 0x00);
  delay_ms(100);
  i2c_reg_write(I2C_IMU, ICM42670_ADDR, 0x1F, 0x0F);
  delay_ms(30);
  i2c_reg_write(I2C_IMU, ICM42670_ADDR, 0x21, 0x68);
  i2c_reg_write(I2C_IMU, ICM42670_ADDR, 0x20, 0x68);
}

/* ========== QMC5883P ========== */

void ahrs_mag_read(ahrs_t *self) {
  uint8_t buf[6] = {0};
  i2c_reg_read_multi(I2C_MAG, QMC5883P_ADDR, 0x01, buf, 6);

  int16_t mx_raw = (buf[1] << 8) | buf[0];
  int16_t my_raw = (buf[3] << 8) | buf[2];
  int16_t mz_raw = (buf[5] << 8) | buf[4];

  self->mag.mx = mx_raw / 2048.0f;
  self->mag.my = my_raw / 2048.0f;
  self->mag.mz = mz_raw / 2048.0f;
  self->mag.mag_norm = sqrtf(self->mag.mx * self->mag.mx +
                             self->mag.my * self->mag.my +
                             self->mag.mz * self->mag.mz);
}

static void qmc5883p_init_hw(void) {
  i2c_reg_write(I2C_MAG, QMC5883P_ADDR, 0x0B, 0x80);
  delay_ms(100);
  i2c_reg_write(I2C_MAG, QMC5883P_ADDR, 0x0B, 0x00);
  delay_ms(100);

  uint8_t id = i2c_reg_read(I2C_MAG, QMC5883P_ADDR, 0x00);
  if (id != 0x80) {
    printf("QMC5883P not detected, ID=0x%02X\r\n", id);
  } else {
    printf("QMC5883P ID OK: 0x%02X\r\n", id);
  }

  i2c_reg_write(I2C_MAG, QMC5883P_ADDR, 0x0B, 0x0C);
  delay_ms(10);
  i2c_reg_write(I2C_MAG, QMC5883P_ADDR, 0x0A, 0x4F);
  delay_ms(10);
}

/* ========== 磁场干扰检测 ========== */

static void mag_disturb_detect(ahrs_t *self) {
  float delta = fabsf(self->mag.mag_norm - self->last_mag_norm);
  self->last_mag_norm = self->mag.mag_norm;
  self->mag_disturb_flag = (delta > self->mag_disturb_thresh) ? 1 : 0;
}

/* ========== 姿态解算 ========== */

void ahrs_calc_6axis(ahrs_t *self, float ax, float ay, float az,
                     float gx, float gy, float gz, float temp) {

  if (self->first_run) {
    self->gx_sum += gx;
    self->gy_sum += gy;
    self->gz_sum += gz;
    self->init_cnt++;

    if (self->init_cnt >= 200) {
      self->gx_bias = self->gx_sum / 200.0f;
      self->gy_bias = self->gy_sum / 200.0f;
      self->gyro_bias.gz_bias = self->gz_sum / 200.0f;
      self->gyro_bias.temp_ref = temp;

      float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.3f;
      float accel_roll  = atan2f(ay, az) * 57.3f;
      euler_to_quat(&self->quat, accel_pitch, accel_roll, 0.0f);
      self->first_run = 0;
    } else {
      self->att.pitch   = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.3f;
      self->att.roll    = atan2f(ay, az) * 57.3f;
      self->att.yaw_now = 0.0f;
      return;
    }
  }

  float gx_comp = gx - self->gx_bias;
  float gy_comp = gy - self->gy_bias;
  float gz_comp = gz - self->gyro_bias.gz_bias;

  uint8_t is_stable =
      (fabsf(gx_comp) < 2.0f) && (fabsf(gy_comp) < 2.0f) && (fabsf(gz_comp) < 2.0f);

  if (is_stable) {
    self->att_stable_cnt++;
  } else {
    self->att_stable_cnt = 0;
    self->ix = 0.0f;
    self->iy = 0.0f;
    self->iz = 0.0f;
  }

  float q0 = self->quat.w, q1 = self->quat.x, q2 = self->quat.y, q3 = self->quat.z;

  float gx_rad = gx_comp * 0.0174533f;
  float gy_rad = gy_comp * 0.0174533f;
  float gz_rad = gz_comp * 0.0174533f;

  float half_dt = self->dt * 0.5f;

  float dq0 = (-q1 * gx_rad - q2 * gy_rad - q3 * gz_rad) * half_dt;
  float dq1 = ( q0 * gx_rad + q2 * gz_rad - q3 * gy_rad) * half_dt;
  float dq2 = ( q0 * gy_rad - q1 * gz_rad + q3 * gx_rad) * half_dt;
  float dq3 = ( q0 * gz_rad + q1 * gy_rad - q2 * gx_rad) * half_dt;

  float acc_norm = sqrtf(ax * ax + ay * ay + az * az);
  if (self->att_stable_cnt > 10 && acc_norm > 0.85f && acc_norm < 1.15f) {
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

    self->ix += ex * Ki;
    self->iy += ey * Ki;
    self->iz += ez * Ki;

    dq1 += (ex * Kp + self->ix) * half_dt;
    dq2 += (ey * Kp + self->iy) * half_dt;
    dq3 += (ez * Kp + self->iz) * half_dt;
  }

  self->quat.w += dq0;
  self->quat.x += dq1;
  self->quat.y += dq2;
  self->quat.z += dq3;

  quat_normalize(&self->quat);

  if (self->att_stable_cnt > 100) {
    self->gx_bias = self->gx_bias * 0.999f + gx * 0.001f;
    self->gy_bias = self->gy_bias * 0.999f + gy * 0.001f;
    self->gyro_bias.gz_bias = self->gyro_bias.gz_bias * 0.999f + gz * 0.001f;
  }

  q0 = self->quat.w; q1 = self->quat.x; q2 = self->quat.y; q3 = self->quat.z;

  float vx = 2.0f * (q1 * q3 - q0 * q2);
  float vy = 2.0f * (q0 * q1 + q2 * q3);
  float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  self->att.pitch   = -atan2f(-vx, sqrtf(vy * vy + vz * vz)) * 57.3f;
  self->att.roll    = -atan2f(vy, vz) * 57.3f;
  self->att.yaw_now += gz_comp * self->dt;

  while (self->att.yaw_now > 180.0f) self->att.yaw_now -= 360.0f;
  while (self->att.yaw_now < -180.0f) self->att.yaw_now += 360.0f;
}

void ahrs_calc_9axis(ahrs_t *self) {
  /* 翻转坐标系适配 9 轴，直接传参给 6 轴解算 */
  ahrs_calc_6axis(self,
    -self->icm.az,  self->icm.ay,  self->icm.ax,
    -self->icm.gz,  self->icm.gy,  self->icm.gx,
     self->icm.temp);

  if (self->day_mode == 0 && self->mag_disturb_flag == 0) {
    float pitch_rad = self->att.pitch * 0.0174533f;
    float cos_p = cosf(pitch_rad);
    if (fabsf(cos_p) > 0.15f) {
      float roll_rad = self->att.roll * 0.0174533f;
      float sin_p = sinf(pitch_rad), cos_r = cosf(roll_rad), sin_r = sinf(roll_rad);

      float mx_h = self->mag.mx * cos_p + self->mag.mz * sin_p;
      float my_h = self->mag.mx * sin_r * sin_p + self->mag.my * cos_r -
                   self->mag.mz * sin_r * cos_p;

      float mag_yaw = atan2f(my_h, mx_h) * 57.3f;
      float yaw_err = mag_yaw - self->att.yaw_now;
      if (yaw_err > 180.0f) yaw_err -= 360.0f;
      if (yaw_err < -180.0f) yaw_err += 360.0f;
      self->att.yaw_now += yaw_err * 0.01f;
    }
  }
}

/* ========== 昼夜模式 ========== */

static void update_day_night_mode(ahrs_t *self, uint8_t rtc_hour) {
  self->day_mode = (rtc_hour >= 6 && rtc_hour < 22) ? 1 : 0;
}

/* ========== Flash 标定 ========== */

int ahrs_save_zero_point(ahrs_t *self) {
  float yaw_sum = 0.0f, pit_sum = 0.0f, rol_sum = 0.0f;
  for (uint16_t i = 0; i < 500; i++) {
    ahrs_icm_read(self);
    ahrs_calc_6axis(self, self->icm.ax, self->icm.ay, self->icm.az,
                     self->icm.gx, self->icm.gy, self->icm.gz,
                     self->icm.temp);
    yaw_sum += self->att.yaw_now;
    pit_sum += self->att.pitch;
    rol_sum += self->att.roll;
    delay_ms(10);
  }

  self->att.yaw_base   = yaw_sum / 500.0f;
  self->att.pitch_base = pit_sum / 500.0f;
  self->att.roll_base  = rol_sum / 500.0f;
  self->gyro_bias.temp_ref = self->icm.temp;

  fmc_unlock();
  fmc_page_erase(FLASH_ZONE_A);
  fmc_word_program(FLASH_ZONE_A,      ((uint32_t *)&self->att)[0]);
  fmc_word_program(FLASH_ZONE_A + 4,  ((uint32_t *)&self->att)[1]);
  fmc_word_program(FLASH_ZONE_A + 8,  ((uint32_t *)&self->att)[2]);
  fmc_word_program(FLASH_ZONE_A + 12, ((uint32_t *)&self->att)[3]);
  fmc_word_program(FLASH_ZONE_A + 16, ((uint32_t *)&self->att)[4]);
  fmc_word_program(FLASH_ZONE_A + 20, ((uint32_t *)&self->att)[5]);
  fmc_word_program(FLASH_ZONE_B,      ((uint32_t *)&self->att)[0]);
  fmc_word_program(FLASH_ZONE_B + 4,  ((uint32_t *)&self->att)[1]);
  fmc_word_program(FLASH_ZONE_B + 8,  ((uint32_t *)&self->att)[2]);
  fmc_word_program(FLASH_ZONE_B + 12, ((uint32_t *)&self->att)[3]);
  fmc_word_program(FLASH_ZONE_B + 16, ((uint32_t *)&self->att)[4]);
  fmc_word_program(FLASH_ZONE_B + 20, ((uint32_t *)&self->att)[5]);
  fmc_word_program(FLASH_GYRO_BIAS,      ((uint32_t *)&self->gyro_bias)[0]);
  fmc_word_program(FLASH_GYRO_BIAS + 4,  ((uint32_t *)&self->gyro_bias)[1]);
  fmc_lock();

  attitude_info_t bak_a, bak_b;
  gyro_bias_t     bak_g;
  flash_read(FLASH_ZONE_A, &bak_a, sizeof(attitude_info_t));
  flash_read(FLASH_ZONE_B, &bak_b, sizeof(attitude_info_t));
  flash_read(FLASH_GYRO_BIAS, &bak_g, sizeof(gyro_bias_t));

  if (bak_a.yaw_base != self->att.yaw_base ||
      bak_b.yaw_base != self->att.yaw_base ||
      bak_a.pitch_base != self->att.pitch_base ||
      bak_b.pitch_base != self->att.pitch_base ||
      bak_a.roll_base != self->att.roll_base ||
      bak_b.roll_base != self->att.roll_base ||
      bak_g.gz_bias != self->gyro_bias.gz_bias ||
      bak_g.temp_ref != self->gyro_bias.temp_ref) {
    return -1;
  }
  return 1;
}

void ahrs_load_zero_point(ahrs_t *self) {
  flash_read(FLASH_ZONE_A, &self->att, sizeof(attitude_info_t));
  flash_read(FLASH_GYRO_BIAS, &self->gyro_bias, sizeof(gyro_bias_t));

  if (isnan(self->att.pitch_base) || isnan(self->att.yaw_now) ||
      self->att.pitch_base > 1000.0f || self->att.pitch_base < -1000.0f) {
    self->att.pitch = 0.0f;
    self->att.roll = 0.0f;
    self->att.pitch_base = 0.0f;
    self->att.roll_base = 0.0f;
    self->att.yaw_base = 0.0f;
    self->att.yaw_now = 0.0f;
    self->gyro_bias.gz_bias = 0.0f;
    self->gyro_bias.temp_ref = 25.0f;
  }
}

/* ========== 传感器初始化 ========== */

void ahrs_sensor_init(ahrs_t *self) {
  icm42670_init_hw();
  qmc5883p_init_hw();
  ahrs_load_zero_point(self);
}

/* ========== 主循环更新 ========== */

void ahrs_update(ahrs_t *self, uint8_t rtc_hour) {
  update_day_night_mode(self, rtc_hour);

  ahrs_icm_read(self);
  ahrs_mag_read(self);
  mag_disturb_detect(self);

  ahrs_calc_6axis(self, self->icm.ax, self->icm.ay, self->icm.az,
                   self->icm.gx, self->icm.gy, self->icm.gz,
                   self->icm.temp);

  /* 报警检测 */
  float yaw_offset = fabsf(self->att.yaw_now - self->att.yaw_base);
  if (yaw_offset > 180.0f) yaw_offset = 360.0f - yaw_offset;
  float pit_offset = fabsf(self->att.pitch - self->att.pitch_base);
  float rol_offset = fabsf(self->att.roll - self->att.roll_base);

  uint8_t trigger = (pit_offset >= self->angle_alarm_threshold ||
                     rol_offset >= self->angle_alarm_threshold ||
                     yaw_offset >= self->angle_alarm_threshold);

  if (trigger) {
    self->alarm_filter_cnt++;
    if (self->alarm_filter_cnt > self->alarm_filter_max) {
      self->fault_type = 0;
      if (pit_offset >= self->angle_alarm_threshold ||
          rol_offset >= self->angle_alarm_threshold)
        self->fault_type = 0x01;
      if (yaw_offset >= self->angle_alarm_threshold)
        self->fault_type = 0x02;
      self->alarm_filter_cnt = self->alarm_filter_max;
    }
  } else {
    self->alarm_filter_cnt = 0;
  }
}
