#include "gd32f3x0_usart.h"
#include "main.h"
#include "stdint.h"
#include <math.h>

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

  buf[idx++] = 0x55;       // head_
  buf[idx++] = 0;          // len_ 暂填
  buf[idx++] = 0x83;       // cmd_ 零位设置

  buf[idx++] = 0x00;       // 零位设置参数

  if (save_install_zero_point() == 1) {
    buf[idx++] = 0x00;     // 成功
  } else {
    buf[idx++] = 0x01;     // 失败
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

  buf[idx++] = 0x55;       // head_
  buf[idx++] = 0;          // len_ 暂填
  buf[idx++] = 0x84;       // cmd_ 零位角度读取

  buf[idx++] = 0x00;       // 读取参数

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

/*总发送分发函数*/
void proto_send(uint8_t cmd) {
  if (usart0_tx_busy) {
    return; // 上一次发送未完成，直接返回
  }
  switch (cmd) {
  default:
    break;
  case 0x82:
    angle_send(att.pitch, att.roll, att.yaw_now, mag_disturb_flag);
    break;
  case 0x83:
    set_zero_angle();
    break;
  case 0x84:
    zero_angle_read_send();
    break;
  };
}