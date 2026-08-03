#ifndef FLASH_H
#define FLASH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== Flash 分区（GD32F330, 每页 1024 字节，共 64 页） ========== */
/*
  0x08000000 - 0x08000FFF  Bootloader (4KB, 第 0-3 页)
  0x08001000 - 0x08007FFF  App 运行区 (32KB, 第 4-11 页)
  0x08008000 - 0x0800DFFF  下载区/OTA (24KB, 第 12-19 页)
  0x0800E000 - 0x0801FFFF  数据分区 (~80KB, 第 20-63 页) — OTA 不碰
*/

#define FLASH_PAGE_SIZE   1024U
#define FLASH_ZONE_A      0x0801FC00U  /* 姿态零点备份 A（第 63 页） */
#define FLASH_ZONE_B      0x0801FD00U  /* 姿态零点备份 B（第 63 页） */
#define FLASH_GYRO_BIAS   0x0801FE00U  /* 陀螺零偏（第 63 页） */
#define FLASH_ALARM_CFG   0x0801F800U  /* 报警参数（第 62 页） */
#define FLASH_YAW_PAGE    0x0800E000U  /* 航向环形缓冲（第 20 页） */
#define FLASH_YAW_SLOTS   256U         /* 每页可存 256 个 float */

/* ========== OTA 分区（ota_protocol.c 使用） ========== */
#define APP_START_ADDR      0x08001000U  /* App 区起始（第 4 页） */
#define APP_END_ADDR        0x08007FFFU  /* App 区结束（第 11 页） */
#define DOWNLOAD_START_ADDR 0x08008000U  /* 下载区起始（第 12 页） */
#define DOWNLOAD_SIZE       0x00006000U  /* 下载区大小 24KB（第 12-19 页） */
#define DATA_ZONE_START     0x0800E000U  /* 数据分区起始（第 20 页），OTA 不碰 */

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