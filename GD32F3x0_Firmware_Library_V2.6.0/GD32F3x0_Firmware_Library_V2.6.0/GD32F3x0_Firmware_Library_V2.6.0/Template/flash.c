#include "flash.h"
#include "main.h"
#include "gd32f3x0.h"
#include <string.h>
#include <math.h>

/* 在 main.c 中引入外部引用，访问全局变量 */
extern icm_raw_data_t   icm_raw;
extern attitude_info_t  att;
extern gyro_bias_t      gyro_bias;

/* ========== 报警参数全局变量定义 ========== */
float    roll_mild_threshold   = 8.0f;
float    pitch_mild_threshold  = 15.0f;
float    yaw_mild_threshold    = 8.0f;
float    roll_severe_threshold = 20.0f;
float    pitch_severe_threshold= 25.0f;
float    yaw_severe_threshold  = 20.0f;
uint16_t alarm_warning_time    = 180;

/* ========== 底层 Flash 读写 ========== */

void flash_read(uint32_t addr, void *data, uint16_t len) {
  memcpy(data, (void *)addr, len);
}

void flash_write(uint32_t addr, void *data, uint16_t len) {
  uint32_t page_addr = addr - (addr % FLASH_PAGE_SIZE);
  uint32_t *src      = (uint32_t *)data;
  uint16_t  word_cnt = len / 4U;

  fmc_unlock();
  fmc_page_erase(page_addr);
  for (uint16_t i = 0; i < word_cnt; i++) {
    fmc_word_program(addr + i * 4, src[i]);
  }
  fmc_lock();
}

/* ========== 外部辅助函数声明 ========== */
extern void     icm42670_get_raw_data(void);
extern void     attitude_calc_6axis(float ax, float ay, float az,
                                    float gx, float gy, float gz, float temp);
extern void     delay_1ms(uint32_t ms);

/* ========== 安装零点 ========== */

int save_install_zero_point(void) {
  float yaw_sum = 0.0f, pit_sum = 0.0f, rol_sum = 0.0f;

  for (uint16_t i = 0; i < 500; i++) {
    icm42670_get_raw_data();
    attitude_calc_6axis(-icm_raw.az, icm_raw.ay, icm_raw.ax,
                        -icm_raw.gz, icm_raw.gy, icm_raw.gx, icm_raw.temp);
    yaw_sum += att.yaw_now;
    pit_sum += att.pitch;
    rol_sum += att.roll;
    delay_1ms(10);
  }

  att.yaw_base       = yaw_sum / 500.0f;
  att.pitch_base     = pit_sum / 500.0f;
  att.roll_base      = rol_sum / 500.0f;
  gyro_bias.temp_ref = icm_raw.temp;

  /* 零点数据写入第 63 页（A/B 双备份 + 陀螺零偏，同页内一次擦除） */
  fmc_unlock();
  fmc_page_erase(FLASH_ZONE_A);
  fmc_word_program(FLASH_ZONE_A,       ((uint32_t *)&att)[0]);
  fmc_word_program(FLASH_ZONE_A  + 4,  ((uint32_t *)&att)[1]);
  fmc_word_program(FLASH_ZONE_A  + 8,  ((uint32_t *)&att)[2]);
  fmc_word_program(FLASH_ZONE_A  + 12, ((uint32_t *)&att)[3]);
  fmc_word_program(FLASH_ZONE_A  + 16, ((uint32_t *)&att)[4]);
  fmc_word_program(FLASH_ZONE_A  + 20, ((uint32_t *)&att)[5]);
  fmc_word_program(FLASH_ZONE_B,       ((uint32_t *)&att)[0]);
  fmc_word_program(FLASH_ZONE_B  + 4,  ((uint32_t *)&att)[1]);
  fmc_word_program(FLASH_ZONE_B  + 8,  ((uint32_t *)&att)[2]);
  fmc_word_program(FLASH_ZONE_B  + 12, ((uint32_t *)&att)[3]);
  fmc_word_program(FLASH_ZONE_B  + 16, ((uint32_t *)&att)[4]);
  fmc_word_program(FLASH_ZONE_B  + 20, ((uint32_t *)&att)[5]);
  fmc_word_program(FLASH_GYRO_BIAS,     ((uint32_t *)&gyro_bias)[0]);
  fmc_word_program(FLASH_GYRO_BIAS + 4, ((uint32_t *)&gyro_bias)[1]);
  fmc_lock();

  /* 读回校验 */
  attitude_info_t bak_a, bak_b;
  gyro_bias_t     bak_g;
  flash_read(FLASH_ZONE_A,     &bak_a, sizeof(attitude_info_t));
  flash_read(FLASH_ZONE_B,     &bak_b, sizeof(attitude_info_t));
  flash_read(FLASH_GYRO_BIAS,  &bak_g, sizeof(gyro_bias_t));

  if (bak_a.yaw_base   != att.yaw_base   ||
      bak_b.yaw_base   != att.yaw_base   ||
      bak_a.pitch_base != att.pitch_base ||
      bak_b.pitch_base != att.pitch_base ||
      bak_a.roll_base  != att.roll_base  ||
      bak_b.roll_base  != att.roll_base  ||
      bak_g.gz_bias    != gyro_bias.gz_bias ||
      bak_g.temp_ref   != gyro_bias.temp_ref) {
    return -1;
  }
  return 1;
}

void load_install_zero_point(void) {
  flash_read(FLASH_ZONE_A,     &att,       sizeof(attitude_info_t));
  flash_read(FLASH_GYRO_BIAS,  &gyro_bias, sizeof(gyro_bias_t));

  if (isnan(att.pitch_base) || isnan(att.yaw_now) ||
      att.pitch_base > 1000.0f || att.pitch_base < -1000.0f) {
    att.pitch       = 0.0f;   att.roll         = 0.0f;
    att.pitch_base  = 0.0f;   att.roll_base    = 0.0f;
    att.yaw_base    = 0.0f;   att.yaw_now      = 0.0f;
    gyro_bias.gz_bias = 0.0f; gyro_bias.temp_ref = 25.0f;
  }
}

/* ========== 报警参数 ========== */

int save_alarm_config(void) {
  alarm_config_t cfg;
  alarm_config_t verify;
  uint32_t *p;
  uint8_t  i;

  cfg.roll_mild    = roll_mild_threshold;
  cfg.pitch_mild   = pitch_mild_threshold;
  cfg.yaw_mild     = yaw_mild_threshold;
  cfg.roll_severe  = roll_severe_threshold;
  cfg.pitch_severe = pitch_severe_threshold;
  cfg.yaw_severe   = yaw_severe_threshold;
  cfg.warning_time = alarm_warning_time;
  cfg.reserved     = 0;

  fmc_unlock();
  fmc_page_erase(FLASH_ALARM_CFG);
  p = (uint32_t *)&cfg;
  for (i = 0; i < sizeof(alarm_config_t) / 4; i++) {
    fmc_word_program(FLASH_ALARM_CFG + i * 4, p[i]);
  }
  fmc_lock();

  flash_read(FLASH_ALARM_CFG, (void *)&verify, sizeof(alarm_config_t));
  if (memcmp(&cfg, &verify, sizeof(alarm_config_t)) != 0) return -1;
  return 1;
}

void load_alarm_config(void) {
  alarm_config_t cfg;
  flash_read(FLASH_ALARM_CFG, (void *)&cfg, sizeof(alarm_config_t));

  if (cfg.roll_mild  < 1.0f || cfg.roll_mild  > 45.0f ||
      cfg.pitch_mild < 1.0f || cfg.pitch_mild > 45.0f ||
      cfg.yaw_mild   < 1.0f || cfg.yaw_mild   > 45.0f ||
      cfg.roll_severe  < cfg.roll_mild  || cfg.roll_severe  > 90.0f ||
      cfg.pitch_severe < cfg.pitch_mild || cfg.pitch_severe > 90.0f ||
      cfg.yaw_severe   < cfg.yaw_mild   || cfg.yaw_severe   > 90.0f ||
      cfg.warning_time < 1 || cfg.warning_time > 3600) {
    /* 恢复默认值 */
    roll_mild_threshold    = 8.0f;
    pitch_mild_threshold   = 15.0f;
    yaw_mild_threshold     = 8.0f;
    roll_severe_threshold  = 20.0f;
    pitch_severe_threshold = 25.0f;
    yaw_severe_threshold   = 20.0f;
    alarm_warning_time     = 180;
  } else {
    roll_mild_threshold    = cfg.roll_mild;
    pitch_mild_threshold   = cfg.pitch_mild;
    yaw_mild_threshold     = cfg.yaw_mild;
    roll_severe_threshold  = cfg.roll_severe;
    pitch_severe_threshold = cfg.pitch_severe;
    yaw_severe_threshold   = cfg.yaw_severe;
    alarm_warning_time     = cfg.warning_time;
  }
}
