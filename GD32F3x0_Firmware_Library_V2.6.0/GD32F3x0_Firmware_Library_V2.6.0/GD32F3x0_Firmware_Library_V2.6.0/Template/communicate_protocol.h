#include "gd32f3x0_usart.h"
#include "hard_i2c.h"
#include "main.h"
#include "stdint.h"
#include <math.h>
#include <stdio.h>


#ifdef __CC_ARM
#define PACKED __packed
#elif defined(__GNUC__)
#define PACKED __attribute__((packed))
#elif defined(__IAR_SYSTEMS_ICC__)
#define PACKED __packed
#else
#define PACKED
#endif

/* 外部声明 */
extern void usart0_tx_dma_send(uint8_t *buf, uint16_t len);
extern volatile uint8_t usart0_tx_busy;
extern uint8_t usart0_rx_buffer[];
extern volatile uint16_t usart0_rx_len;

/* 缓冲区 */
static uint8_t buf[256];

/*校验*/
uint8_t calc_checksum(const uint8_t *data, uint16_t len) {
  uint8_t xor_val = 0;
  for (uint16_t i = 0; i < len; i++) {
    xor_val ^= data[i];
  }
  return ~xor_val;
}

/*角度读取*/
// 0x82
void angle_send(float pitch, float roll, float yaw, uint8_t mag_disturb_flag) {
  uint16_t idx = 0;

  // ---- 填充头部 ----
  buf[idx++] = 0x55; // head_
  buf[idx++] = 0;    // len_ 暂填，最后计算     // len_
  buf[idx++] = 0x82; // cmd_

  // ---- 填充数据 ----
  // uint8_t pram_idx = idx;
  buf[idx++] = 0x00; // pram_

  buf[idx++] = (roll < 0) ? 0x80 : 0x00;
  buf[idx++] = (uint8_t)((uint16_t)(fabsf(roll) * 100) >> 8);
  buf[idx++] = (uint8_t)((uint16_t)(fabsf(roll) * 100) & 0xFF);

  buf[idx++] = (pitch < 0) ? 0x80 : 0x00;
  buf[idx++] = (uint8_t)((uint16_t)(fabsf(pitch) * 100) >> 8);
  buf[idx++] = (uint8_t)((uint16_t)(fabsf(pitch) * 100) & 0xFF);

  buf[idx++] = (yaw < 0) ? 0x80 : 0x00;
  buf[idx++] = (uint8_t)((uint16_t)(fabsf(yaw) * 100) >> 8);
  buf[idx++] = (uint8_t)((uint16_t)(fabsf(yaw) * 100) & 0xFF);

  buf[idx++] = mag_disturb_flag ? 1 : 0; // if_mag_disturb_
  buf[idx++] =
      (uint8_t)((uint16_t)(icm_raw.temp * 100) >> 8); // 温度值，单位摄氏度
  buf[idx++] = (uint8_t)((uint16_t)(icm_raw.temp * 100) & 0xFF);
  // ---- 计算并填充帧长 & 校验码 ----
  uint8_t frame_len = (uint8_t)(idx - 1); // 从 len_ 到最后一个数据字节数
  buf[1] = frame_len;                     // 填入 len_

  uint8_t cs =
      calc_checksum(&buf[1], frame_len); // 校验范围：从 len_ 到 if_mag_disturb_
  buf[idx++] = cs;                       // check_

  // ---- DMA 发送（非阻塞） ----
  usart0_tx_dma_send(buf, idx);
}

/*零位设置状态返回（协议格式同 angle_send）*/
// 0x83
void set_zero_angle() {
  uint16_t idx = 0;

  buf[idx++] = 0x55; // head_
  buf[idx++] = 0;    // len_ 暂填
  buf[idx++] = 0x83; // cmd_ 零位设置

  buf[idx++] = 0x00; // 零位设置参数

  if (save_install_zero_point() == 1) {
    buf[idx++] = 0x00; // 成功
  } else {
    buf[idx++] = 0x01; // 失败
  }

  // ---- 计算并填充帧长 & 校验码 ----
  uint8_t frame_len = (uint8_t)(idx - 1);
  buf[1] = frame_len;

  uint8_t cs = calc_checksum(&buf[1], frame_len);
  buf[idx++] = cs;

  // ---- DMA 发送 ----
  usart0_tx_dma_send(buf, idx);
}

/*零位角度读取返回（返回安装基准零点 pitch_base/roll_base/yaw_base）*/
// 0x84
void zero_angle_read_send() {
  uint16_t idx = 0;

  buf[idx++] = 0x55; // head_
  buf[idx++] = 0;    // len_ 暂填
  buf[idx++] = 0x84; // cmd_ 零位角度读取

  buf[idx++] = 0x00; // 读取参数

  // Roll (att.roll_base)
  buf[idx++] = (att.roll_base < 0) ? 0x80 : 0x00;
  buf[idx++] = (uint8_t)((uint16_t)(fabsf(att.roll_base) * 100) >> 8);
  buf[idx++] = (uint8_t)((uint16_t)(fabsf(att.roll_base) * 100) & 0xFF);

  // Pitch (att.pitch_base)
  buf[idx++] = (att.pitch_base < 0) ? 0x80 : 0x00;
  buf[idx++] = (uint8_t)((uint16_t)(fabsf(att.pitch_base) * 100) >> 8);
  buf[idx++] = (uint8_t)((uint16_t)(fabsf(att.pitch_base) * 100) & 0xFF);

  // Yaw (att.yaw_base)
  buf[idx++] = (att.yaw_base < 0) ? 0x80 : 0x00;
  buf[idx++] = (uint8_t)((uint16_t)(fabsf(att.yaw_base) * 100) >> 8);
  buf[idx++] = (uint8_t)((uint16_t)(fabsf(att.yaw_base) * 100) & 0xFF);

  // ---- 计算并填充帧长 & 校验码 ----
  uint8_t frame_len = (uint8_t)(idx - 1);
  buf[1] = frame_len;

  uint8_t cs = calc_checksum(&buf[1], frame_len);
  buf[idx++] = cs;

  // ---- DMA 发送 ----
  usart0_tx_dma_send(buf, idx);
}

// 0x85偏转阈值设置
/* 上位机帧头 0xAA，本机回复帧头 0x55 */
static void threshold_set(void) {
  uint8_t param;
  float x_val, y_val, z_val;
  int save_ok;
  uint16_t idx;
  uint8_t frame_len;
  uint8_t cs;

  // 解析接收帧：AA | 06 | 85 | param | X | Y | Z | cs
  if (usart0_rx_len < 8)
    return;

  param = usart0_rx_buffer[3];
  x_val = (float)usart0_rx_buffer[4];
  y_val = (float)usart0_rx_buffer[5];
  z_val = (float)usart0_rx_buffer[6];

  // 校验范围 1°~90°
  if (x_val < 1.0f || x_val > 90.0f || y_val < 1.0f || y_val > 90.0f ||
      z_val < 1.0f || z_val > 90.0f) {
    return;
  }

  if (param == 0x00) {
    roll_mild_threshold = x_val;
    pitch_mild_threshold = y_val;
    yaw_mild_threshold = z_val;
    if (roll_severe_threshold > 0.0f &&
        roll_mild_threshold >= roll_severe_threshold)
      return;
    if (pitch_severe_threshold > 0.0f &&
        pitch_mild_threshold >= pitch_severe_threshold)
      return;
    if (yaw_severe_threshold > 0.0f &&
        yaw_mild_threshold >= yaw_severe_threshold)
      return;
    save_ok = save_alarm_config();
  } else if (param == 0x01) {
    roll_severe_threshold = x_val;
    pitch_severe_threshold = y_val;
    yaw_severe_threshold = z_val;
    if (roll_mild_threshold > 0.0f &&
        roll_severe_threshold <= roll_mild_threshold)
      return;
    if (pitch_mild_threshold > 0.0f &&
        pitch_severe_threshold <= pitch_mild_threshold)
      return;
    if (yaw_mild_threshold > 0.0f && yaw_severe_threshold <= yaw_mild_threshold)
      return;
    save_ok = save_alarm_config();
  } else {
    return;
  }

  // 回复帧：55 | 04 | 85 | param | status | cs
  idx = 0;
  buf[idx++] = 0x55;
  buf[idx++] = 0;
  buf[idx++] = 0x85;
  buf[idx++] = param;
  buf[idx++] = (save_ok == 1) ? 0x00 : 0x01;
  frame_len = (uint8_t)(idx - 1);
  buf[1] = frame_len;
  cs = calc_checksum(&buf[1], frame_len);
  buf[idx++] = cs;
  usart0_tx_dma_send(buf, idx);
}

/*偏转报警状态读取返回*/
void alarm_status_read_send() {
  uint16_t idx = 0;

  buf[idx++] = 0x55; // head_
  buf[idx++] = 0;    // len_ 暂填
  buf[idx++] = 0x87; // cmd_ 偏转报警状态读取

  buf[idx++] = 0x00;       // 读取参数
  buf[idx++] = fault_type; // 状态

  // ---- 计算并填充帧长 & 校验码 ----
  uint8_t frame_len = (uint8_t)(idx - 1);
  buf[1] = frame_len;

  uint8_t cs = calc_checksum(&buf[1], frame_len);
  buf[idx++] = cs;

  // ---- DMA 发送 ----
  usart0_tx_dma_send(buf, idx);
}

/*读取地磁强度返回（使用 mag_raw.mag_norm 及三轴分量）*/
void mag_strength_read_send() {
  uint16_t idx = 0;

  buf[idx++] = 0x55; // head_
  buf[idx++] = 0;    // len_ 暂填
  buf[idx++] = 0x8A; // cmd_ 读取地磁强度

  buf[idx++] = 0x00; // 读取参数

  // mag_norm 整数值
  uint16_t mag_val =
      (uint16_t)(mag_raw.mag_norm * 100.0f); // 先乘100保留两位小数，再转 uint16
  buf[idx++] = (uint8_t)(mag_val >> 8);
  buf[idx++] = (uint8_t)(mag_val & 0xFF);

  // // 三轴分量：先乘100保留两位小数，再转 int16
  // int16_t mx_i = (int16_t)(mag_raw.mx * 100.0f);
  // int16_t my_i = (int16_t)(mag_raw.my * 100.0f);
  // int16_t mz_i = (int16_t)(mag_raw.mz * 100.0f);
  // buf[idx++] = (mx_i < 0) ? 0x80 : 0x00;
  // buf[idx++] = (uint8_t)((uint16_t)(mx_i < 0 ? -mx_i : mx_i) >> 8);
  // buf[idx++] = (uint8_t)((uint16_t)(mx_i < 0 ? -mx_i : mx_i) & 0xFF);
  // buf[idx++] = (my_i < 0) ? 0x80 : 0x00;
  // buf[idx++] = (uint8_t)((uint16_t)(my_i < 0 ? -my_i : my_i) >> 8);
  // buf[idx++] = (uint8_t)((uint16_t)(my_i < 0 ? -my_i : my_i) & 0xFF);
  // buf[idx++] = (mz_i < 0) ? 0x80 : 0x00;
  // buf[idx++] = (uint8_t)((uint16_t)(mz_i < 0 ? -mz_i : mz_i) >> 8);
  // buf[idx++] = (uint8_t)((uint16_t)(mz_i < 0 ? -mz_i : mz_i) & 0xFF);

  // ---- 计算并填充帧长 & 校验码 ----
  uint8_t frame_len = (uint8_t)(idx - 1);
  buf[1] = frame_len;

  uint8_t cs = calc_checksum(&buf[1], frame_len);
  buf[idx++] = cs;

  // ---- DMA 发送 ----
  usart0_tx_dma_send(buf, idx);
}

/*收到主动上报信息回复*/
void active_report_ack_send() {
  uint16_t idx = 0;

  buf[idx++] = 0x55; // head_
  buf[idx++] = 0;    // len_ 暂填
  buf[idx++] = 0x8D; // cmd_ 收到主动上报

  buf[idx++] = 0x00; // 参数
  buf[idx++] = 0x00; // 状态：成功

  // ---- 计算并填充帧长 & 校验码 ----
  uint8_t frame_len = (uint8_t)(idx - 1);
  buf[1] = frame_len;

  uint8_t cs = calc_checksum(&buf[1], frame_len);
  buf[idx++] = cs;

  // ---- DMA 发送 ----
  usart0_tx_dma_send(buf, idx);
}

/*心跳帧（功能码 0x8E，含递增计数 + I2C 设备在线状态）*/
void heartbeat_send() {
  static uint8_t counter = 0;
  uint16_t idx = 0;

  buf[idx++] = 0x55; // head_
  buf[idx++] = 0;    // len_ 暂填
  buf[idx++] = 0x8E; // cmd_ 心跳

  buf[idx++] = 0x00;      // 参数
  buf[idx++] = counter++; // 心跳计数（每帧+1，溢出自动归零）

  /* 探测 I2C 从设备是否在线（0=在线，1=离线）*/
  buf[idx++] = hard_i2c_probe(0x68); // IMU(ICM42670) 状态
  buf[idx++] = hard_i2c_probe(0x2C); // MAG(QMC5883P) 状态

  uint8_t frame_len = (uint8_t)(idx - 1);
  buf[1] = frame_len;

  uint8_t cs = calc_checksum(&buf[1], frame_len);
  buf[idx++] = cs;

  usart0_tx_dma_send(buf, idx);
}

/*读取偏转阈值返回（cmd 0x86）*/
/* 上位机帧头 0xAA，本机回复帧头 0x55 */
/* 接收：AA | 03 | 86 | param | cs          (param=0x00轻微 0x01严重) */
/* 回复：55 | 06 | 86 | param | X | Y | Z | cs */
static void threshold_read_send(void) {
  uint16_t idx;
  uint8_t param;
  uint8_t frame_len;
  uint8_t cs;

  param = usart0_rx_buffer[3];

  idx = 0;
  buf[idx++] = 0x55;
  buf[idx++] = 0;
  buf[idx++] = 0x86;
  buf[idx++] = param;

  if (param == 0x00) {
    buf[idx++] = (uint8_t)roll_mild_threshold;
    buf[idx++] = (uint8_t)pitch_mild_threshold;
    buf[idx++] = (uint8_t)yaw_mild_threshold;
  } else if (param == 0x01) {
    buf[idx++] = (uint8_t)roll_severe_threshold;
    buf[idx++] = (uint8_t)pitch_severe_threshold;
    buf[idx++] = (uint8_t)yaw_severe_threshold;
  }

  frame_len = (uint8_t)(idx - 1);
  buf[1] = frame_len;
  cs = calc_checksum(&buf[1], frame_len);
  buf[idx++] = cs;
  usart0_tx_dma_send(buf, idx);
}

/*偏转预警时间设置（cmd 0x88）*/
/* 上位机帧头 0xAA，本机回复帧头 0x55 */
/* 接收：AA | 05 | 88 | 00 | time_H | time_L | cs */
/* 回复：55 | 04 | 88 | 00 | status | cs */
static void warning_time_set(void) {
  uint16_t time_sec;
  int save_ok;
  uint16_t idx;
  uint8_t frame_len;
  uint8_t cs;

  if (usart0_rx_len < 7)
    return;

  time_sec = (usart0_rx_buffer[4] << 8) | usart0_rx_buffer[5];

  if (time_sec < 1 || time_sec > 3600)
    return;

  alarm_warning_time = time_sec;
  save_ok = save_alarm_config();

  idx = 0;
  buf[idx++] = 0x55;
  buf[idx++] = 0;
  buf[idx++] = 0x88;
  buf[idx++] = 0x00;
  buf[idx++] = (save_ok == 1) ? 0x00 : 0x01;
  frame_len = (uint8_t)(idx - 1);
  buf[1] = frame_len;
  cs = calc_checksum(&buf[1], frame_len);
  buf[idx++] = cs;
  usart0_tx_dma_send(buf, idx);
}

/*读取偏转预警时间返回（cmd 0x89）*/
/* 上位机帧头 0xAA，本机回复帧头 0x55 */
/* 接收：AA | 03 | 89 | 00 | cs */
/* 回复：55 | 05 | 89 | 00 | time_H | time_L | cs */
static void warning_time_read_send(void) {
  uint16_t idx = 0;
  uint8_t frame_len;
  uint8_t cs;

  buf[idx++] = 0x55;
  buf[idx++] = 0;
  buf[idx++] = 0x89;
  buf[idx++] = 0x00;
  buf[idx++] = (uint8_t)(alarm_warning_time >> 8);
  buf[idx++] = (uint8_t)(alarm_warning_time & 0xFF);

  frame_len = (uint8_t)(idx - 1);
  buf[1] = frame_len;
  cs = calc_checksum(&buf[1], frame_len);
  buf[idx++] = cs;
  usart0_tx_dma_send(buf, idx);
}

void proto_send(uint8_t cmd) {
  if (usart0_tx_busy) {
    return;
  }

  switch (cmd) {
  case 0x82:
    angle_send(att.pitch, att.roll, att.yaw_now, mag_disturb_flag);
    break;
  case 0x83:
    set_zero_angle();
    break;
  case 0x84:
    zero_angle_read_send();
    break;
  case 0x85:
    threshold_set();
    break;
  case 0x86:
    threshold_read_send();
    break;
  case 0x87:
    alarm_status_read_send();
    break;
  case 0x88:
    warning_time_set();
    break;
  case 0x89:
    warning_time_read_send();
    break;
  case 0x8A:
    mag_strength_read_send();
    break;
  case 0x8D:
    active_report_ack_send();
    break;
  case 0x8E:
    heartbeat_send();
    break;
  }
}