#ifndef FLASH_H
#define FLASH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== Flash 分区（GD32F330, 每页 1024 字节） ========== */

#define FLASH_PAGE_SIZE   1024U
#define FLASH_ZONE_A      0x0800FC00U  /* 姿态零点备份 A（第 63 页） */
#define FLASH_ZONE_B      0x0800FD00U  /* 姿态零点备份 B（第 63 页） */
#define FLASH_GYRO_BIAS   0x0800FE00U  /* 陀螺零偏（第 63 页） */
#define FLASH_ALARM_CFG   0x0800F800U  /* 报警参数（第 62 页） */
#define FLASH_YAW_PAGE    0x0800EC00U  /* 航向环形缓冲（第 59 页） */
#define FLASH_YAW_SLOTS   256U         /* 每页可存 256 个 float */

/* ========== 报警参数结构体 ========== */

typedef struct {
  float    roll_mild;
  float    pitch_mild;
  float    yaw_mild;
  float    roll_severe;
  float    pitch_severe;
  float    yaw_severe;
  uint16_t warning_time;
  uint16_t reserved;
} alarm_config_t;

/* ========== 报警参数全局变量 ========== */

extern float    roll_mild_threshold;
extern float    pitch_mild_threshold;
extern float    yaw_mild_threshold;
extern float    roll_severe_threshold;
extern float    pitch_severe_threshold;
extern float    yaw_severe_threshold;
extern uint16_t alarm_warning_time;

/* ========== API ========== */

/* 底层 Flash 读写 */
void flash_read(uint32_t addr, void *data, uint16_t len);
void flash_write(uint32_t addr, void *data, uint16_t len);

/* 安装零点（保存 / 加载） */
int  save_install_zero_point(void);
void load_install_zero_point(void);

/* 报警参数（保存 / 加载） */
int  save_alarm_config(void);
void load_alarm_config(void);

/* 航向角断电保存（环形缓冲，每 10ms 调用一次） */
void save_yaw_to_flash(float current_yaw);
void load_yaw_from_flash(void);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_H */