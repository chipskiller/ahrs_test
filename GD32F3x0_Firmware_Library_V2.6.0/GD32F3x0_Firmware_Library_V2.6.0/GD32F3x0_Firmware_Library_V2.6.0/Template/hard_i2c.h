#ifndef HARD_I2C_H
#define HARD_I2C_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* I2C1 引脚定义 */
#define HARD_I2C_SCL_PIN GPIO_PIN_0
#define HARD_I2C_SDA_PIN GPIO_PIN_1
#define HARD_I2C_PORT    GPIOA
#define HARD_I2C_PERIPH  I2C1
#define HARD_I2C_SPEED   400000  /* 400kHz 快速模式：提速4倍以满足500Hz主循环2ms预算 */

/* I2C 超时计数 */
#define I2C_TIMEOUT 10000U

/* I2C 状态机状态 */
#define I2C_START           0
#define I2C_SEND_ADDRESS    1
#define I2C_CLEAR_ADDRESS_FLAG 2
#define I2C_TRANSMIT_DATA   3
#define I2C_RECEIVE_DATA    4
#define I2C_STOP            5
#define I2C_END             6

/* 函数声明 */
void hard_i2c_init(void);
uint8_t i2c_reg_read(uint32_t i2c_periph, uint8_t dev_addr, uint8_t reg);
void i2c_reg_write(uint32_t i2c_periph, uint8_t dev_addr, uint8_t reg, uint8_t data);
void i2c_reg_read_multi(uint32_t i2c_periph, uint8_t dev_addr, uint8_t reg,
                        uint8_t *buf, uint16_t len);
uint8_t hard_i2c_probe(uint8_t dev_addr);

#ifdef __cplusplus
}
#endif

#endif /* HARD_I2C_H */