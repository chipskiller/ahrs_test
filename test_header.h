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
extern uint8_t usart0_rx_buffer[];
extern volatile uint16_t usart0_rx_len;

/* 缓冲区 */
static uint8_t buf[256];

/*校验*/
uint8_t calc_checksum(const uint8_t *data, uint16_t len) {
  uint8_t xor_val = 0;
  uint16_t i;
  for (i = 0; i < len; i++) {
    xor_val ^= data[i];
  }
  return ~xor_val;
}

/*角度读取（含温度）*/
void angle_send(float pitch, float roll, float yaw, uint8_t mag_disturb_flag) {
  uint16_t idx = 0;
  uint8_t frame_len;
  uint8_t cs;

  // ---- 填充头部 ----
  buf[idx++] = 0x55;
  buf[idx++] = 0;
  buf[idx++] = 0x82;

  buf[idx++] = 0x00;

  buf[idx++] = (roll < 0) ? 0x80 : 0x00;
  buf[idx++] = (uint8_t)((uint16_t)(fabsf(roll) * 100) >> 8);
  buf[idx++] = (uint8_t)((uint16_t)(fabsf(roll) * 100) & 0xFF);

  buf[idx++] = (pitch < 0) ? 0x80 : 0x00;
  buf[idx++] = (uint8_t)((uint16_t)(fabsf(pitch) * 100) >> 8);
  buf[idx++] = (uint8_t)((uint16_t)(fabsf(pitch) * 100) & 0xFF);

  buf[idx++] = (yaw < 0) ? 0x80 : 0x00;
  buf[idx++] = (uint8_t)((uint16_t)(fabsf(yaw) * 100) >> 8);
  buf[idx++] = (uint8_t)((uint16_t)(fabsf(yaw) * 100) & 0xFF);

  buf[idx++] = mag_disturb_flag ? 1 : 0;
  buf[idx++] = (uint8_t)((uint16_t)(icm_raw.temp * 100) >> 8);
  buf[idx++] = (uint8_t)((uint16_t)(icm_raw.temp * 100) & 0xFF);

  frame_len = (uint8_t)(idx - 1);
  buf[1] = frame_len;
  cs = calc_checksum(&buf[1], frame_len);
  buf[idx++] = cs;
  usart0_tx_dma_send(buf, idx);
}
