#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* I2C 引脚定义（可根据需要修改） */
#define SOFT_I2C_SCL_PIN GPIO_PIN_0
#define SOFT_I2C_SDA_PIN GPIO_PIN_1
#define SOFT_I2C_PORT    GPIOA

/* GPIO 位操作宏 */
#define SCL_H() gpio_bit_set(SOFT_I2C_PORT, SOFT_I2C_SCL_PIN)
#define SCL_L() gpio_bit_reset(SOFT_I2C_PORT, SOFT_I2C_SCL_PIN)
#define SDA_H() gpio_bit_set(SOFT_I2C_PORT, SOFT_I2C_SDA_PIN)
#define SDA_L() gpio_bit_reset(SOFT_I2C_PORT, SOFT_I2C_SDA_PIN)
#define SDA_IN() gpio_input_bit_get(SOFT_I2C_PORT, SOFT_I2C_SDA_PIN)

/* ---- 传输层 ---- */
void soft_i2c_init(void);           /* 初始化 GPIO 为开漏输出 */
void soft_i2c_start(void);          /* 产生 START 条件 */
void soft_i2c_stop(void);           /* 产生 STOP 条件 */
uint8_t soft_i2c_send_byte(uint8_t byte);  /* 发 8 位 + 读 ACK，返回 0=ACK / 1=NACK */
uint8_t soft_i2c_read_byte(uint8_t ack);   /* 读 8 位 + 发 ACK(1)/NACK(0) */

/* ---- 应用层 ---- */
uint8_t i2c_reg_read(uint32_t i2c_periph, uint8_t dev_addr, uint8_t reg);
void   i2c_reg_write(uint32_t i2c_periph, uint8_t dev_addr, uint8_t reg, uint8_t data);
void   i2c_reg_read_multi(uint32_t i2c_periph, uint8_t dev_addr, uint8_t reg,
                          uint8_t *buf, uint16_t len);
uint8_t soft_i2c_probe(uint8_t dev_addr);  /* 探测设备：返回 0=在线(ACK) / 1=离线(NACK) */

#ifdef __cplusplus
}
#endif

#endif /* SOFT_I2C_H */