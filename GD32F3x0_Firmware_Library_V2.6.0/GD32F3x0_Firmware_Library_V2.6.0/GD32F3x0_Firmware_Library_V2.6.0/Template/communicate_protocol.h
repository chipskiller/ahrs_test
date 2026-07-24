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

/* 外部函数声明 */
extern void usart0_tx_dma_send(uint8_t *buf, uint16_t len);
extern volatile uint8_t usart0_tx_busy;

/* 协议通用定义 */

// typedef PACKED struct {
//   uint8_t head_; // 帧头 0xAA/0x55
//   uint8_t len_;  // 从本字节到 checksum 的长度
//   uint8_t cmd_;  // 功能码
// } proto_head_t;

uint8_t calc_checksum(const uint8_t *data, uint16_t len) {
  uint8_t xor_val = 0;
  for (uint16_t i = 0; i < len; i++) {
    xor_val ^= data[i];
  }
  return ~xor_val;
}

/*角度读取*/

// typedef PACKED struct {
//   uint8_t pram_;
//   uint8_t check_;
// } angleget_frame_t;

// typedef PACKED struct {
//   uint8_t pram_;
//   uint8_t roll_sign_;
//   uint8_t roll_data_high_;
//   uint8_t roll_data_low_;
//   uint8_t pitch_sign_;
//   uint8_t pitch_data_high_;
//   uint8_t pitch_data_low_;
//   uint8_t yaw_sign_;
//   uint8_t yaw_data_high_;
//   uint8_t yaw_data_low_;
//   uint8_t if_mag_disturb_;
//   uint8_t check_;
// } anglesend_frame_t;

void angle_send(float pitch, float roll, float yaw, uint8_t mag_disturb_flag) {
  static uint8_t buf[14]; // 连续缓冲区（static 防止 DMA 非阻塞发送时栈回收）
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

void proto_send(uint8_t cmd) {
  if (usart0_tx_busy) {
    return; // 上一次发送未完成，直接返回
  }
  switch (cmd) {
  default:
    break;
  case 0x82:
    angle_send(att.pitch, att.roll, att.yaw_now, mag_disturb_flag);
    // printf("\r\nsend angle: P=%.2f,R=%.2f,Y=%.2f,mag_disturb=%d\r\n", att.pitch,
    //        att.roll, att.yaw_now, mag_disturb_flag);
    break;
  };
}