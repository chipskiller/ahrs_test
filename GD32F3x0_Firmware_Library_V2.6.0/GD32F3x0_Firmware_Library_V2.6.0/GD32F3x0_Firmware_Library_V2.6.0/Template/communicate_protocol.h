#include "gd32f3x0_usart.h"
#include "main.h"
#include "soft_i2c.h"
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

/*读取地磁强度返回（使用 mag_raw.mag_norm）*/
void mag_strength_read_send() {
  uint16_t idx = 0;

  uint16_t mag_val = (uint16_t)(mag_raw.mag_norm); // float 转 uint16

  buf[idx++] = 0x55; // head_
  buf[idx++] = 0;    // len_ 暂填
  buf[idx++] = 0x8A; // cmd_ 读取地磁强度

  buf[idx++] = 0x00;                      // 读取参数
  buf[idx++] = (uint8_t)(mag_val >> 8);   // 数据高位
  buf[idx++] = (uint8_t)(mag_val & 0xFF); // 数据低位

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

  buf[idx++] = 0x55;       // head_
  buf[idx++] = 0;          // len_ 暂填
  buf[idx++] = 0x8E;       // cmd_ 心跳

  buf[idx++] = 0x00;       // 参数
  buf[idx++] = counter++;  // 心跳计数（每帧+1，溢出自动归零）

  /* 探测 I2C 从设备是否在线（0=在线，1=离线）*/
  buf[idx++] = soft_i2c_probe(0x68);  // IMU(ICM42670) 状态
  buf[idx++] = soft_i2c_probe(0x2C);  // MAG(QMC5883P) 状态

  uint8_t frame_len = (uint8_t)(idx - 1);
  buf[1] = frame_len;

  uint8_t cs = calc_checksum(&buf[1], frame_len);
  buf[idx++] = cs;

  usart0_tx_dma_send(buf, idx);
}

/*自动上报（轮询发送 + 心跳，非阻塞降频）*/
void proto_send(uint8_t cmd) {
  if (usart0_tx_busy) {
    return;
  }

  static uint8_t seq = 0;
  switch (seq) {
  case 0:
    angle_send(att.pitch, att.roll, att.yaw_now, mag_disturb_flag);
    break;
  case 1:
    heartbeat_send();
    break;
  }
  seq++;
  if (seq >= 2) seq = 0;
}