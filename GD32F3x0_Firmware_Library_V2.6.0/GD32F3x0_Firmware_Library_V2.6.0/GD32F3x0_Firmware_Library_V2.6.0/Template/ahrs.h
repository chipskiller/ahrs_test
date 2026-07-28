#ifndef AHRS_H
#define AHRS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 类型定义 ========== */

typedef struct {
  float w, x, y, z;
} quaternion_t;

typedef struct {
  float gz_bias;
  float temp_ref;
} gyro_bias_t;

typedef struct {
  float pitch;
  float roll;
  float yaw_now;
  float pitch_base;
  float roll_base;
  float yaw_base;
} attitude_info_t;

typedef struct {
  float mx;
  float my;
  float mz;
  float mag_norm;
} mag_raw_data_t;

typedef struct {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  float temp;
} icm_raw_data_t;

/* ========== AHRS 主结构体（类似 C++ class） ========== */

typedef struct {
  /* ---- 传感器数据 ---- */
  icm_raw_data_t  icm;
  mag_raw_data_t  mag;
  attitude_info_t att;
  gyro_bias_t     gyro_bias;
  quaternion_t    quat;

  /* ---- 状态标志 ---- */
  uint8_t day_mode;          /* 1=白天6轴  0=夜间9轴 */
  uint8_t mag_disturb_flag;  /* 地磁干扰标记 */
  uint8_t fault_type;        /* 偏转报警类型 */

  /* ---- 姿态解算内部状态（原 static 局部变量） ---- */
  uint8_t  first_run;
  float    gx_bias, gy_bias;
  float    gx_sum, gy_sum, gz_sum;
  uint16_t init_cnt;
  float    ix, iy, iz;
  uint16_t att_stable_cnt;
  float    last_mag_norm;

  /* ---- 报警防抖 ---- */
  uint16_t alarm_filter_cnt;

  /* ---- 配置参数 ---- */
  float    dt;
  float    angle_alarm_threshold;
  uint16_t alarm_filter_max;
  float    mag_disturb_thresh;
} ahrs_t;

/* ========== API（类似 C++ 成员函数） ========== */

/* 初始化 AHRS 对象 */
void ahrs_init(ahrs_t *self);

/* 传感器初始化（ICM42670 + QMC5883P） */
void ahrs_sensor_init(ahrs_t *self);

/* 10ms 主循环更新（采集 → 解算 → 报警检测） */
void ahrs_update(ahrs_t *self, uint8_t rtc_hour);

/* 单独采集传感器原始数据 */
void ahrs_icm_read(ahrs_t *self);
void ahrs_mag_read(ahrs_t *self);

/* 姿态解算（IMU 数据由参数传入，与结构体解耦） */
void ahrs_calc_6axis(ahrs_t *self, float ax, float ay, float az,
                     float gx, float gy, float gz, float temp);
void ahrs_calc_9axis(ahrs_t *self);

/* Flash 标定零点 */
int  ahrs_save_zero_point(ahrs_t *self);
void ahrs_load_zero_point(ahrs_t *self);

#ifdef __cplusplus
}
#endif

#endif /* AHRS_H */
