#include "gd32f3x0.h"
#include "systick.h"
#include "soft_i2c.h"

/* 外部延时函数（定义在 systick.c，声明在 systick.h）*/

/*!
    \brief      初始化 GPIO 为开漏输出
    \param[in]  无
    \retval     无
*/
void soft_i2c_init(void) {
    rcu_periph_clock_enable(RCU_GPIOA);

    gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,
                  GPIO_PIN_0 | GPIO_PIN_1);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,
                            GPIO_PIN_0 | GPIO_PIN_1);

    SCL_H();
    SDA_H();
    delay_1ms(10);
}

/*!
    \brief      产生 I2C 起始信号
    \param[in]  无
    \retval     无
*/
void soft_i2c_start(void) {
    SDA_H();
    SCL_H();
    delay_us(5);
    SDA_L();
    delay_us(5);
    SCL_L();
    delay_us(5);
}

/*!
    \brief      产生 I2C 停止信号
    \param[in]  无
    \retval     无
*/
void soft_i2c_stop(void) {
    SDA_L();
    SCL_H();
    delay_us(5);
    SDA_H();
    delay_us(5);
}

/*!
    \brief      发送一个字节（MSB 优先），读 ACK
    \param[in]  byte: 待发送数据
    \retval     0 = 收到 ACK, 1 = 收到 NACK
*/
uint8_t soft_i2c_send_byte(uint8_t byte) {
    uint8_t i;

    for (i = 0; i < 8; i++) {
        SCL_L();
        delay_us(3);
        if (byte & 0x80) {
            SDA_H();
        } else {
            SDA_L();
        }
        delay_us(2);
        SCL_H();
        delay_us(5);
        byte <<= 1;
    }

    SCL_L();
    delay_us(3);
    SDA_H();
    delay_us(2);
    SCL_H();
    delay_us(5);

    uint8_t ack = SDA_IN();

    SCL_L();
    delay_us(5);

    return (ack == 0) ? 0 : 1;
}

/*!
    \brief      读取一个字节
    \param[in]  ack: 1-发送 ACK(继续读), 0-发送 NACK(停止)
    \retval     读取到的数据
*/
uint8_t soft_i2c_read_byte(uint8_t ack) {
    uint8_t i, byte = 0;

    SDA_H();

    for (i = 0; i < 8; i++) {
        SCL_L();
        delay_us(5);
        SCL_H();
        delay_us(3);
        byte <<= 1;
        if (SDA_IN()) {
            byte |= 0x01;
        }
        delay_us(2);
    }

    SCL_L();
    delay_us(3);
    if (ack) {
        SDA_L();
    } else {
        SDA_H();
    }
    delay_us(2);
    SCL_H();
    delay_us(5);

    SCL_L();
    delay_us(5);

    return byte;
}

/*!
    \brief      读取单个寄存器
    \param[in]  i2c_periph: I2C 外设（软件模拟，保留参数为兼容硬件 I2C）
    \param[in]  dev_addr: 设备 7 位地址
    \param[in]  reg: 寄存器地址
    \retval     读取到的寄存器值
*/
uint8_t i2c_reg_read(uint32_t i2c_periph, uint8_t dev_addr, uint8_t reg) {
    soft_i2c_start();
    soft_i2c_send_byte((dev_addr << 1) | 0);
    soft_i2c_send_byte(reg);
    soft_i2c_start();
    soft_i2c_send_byte((dev_addr << 1) | 1);
    uint8_t data = soft_i2c_read_byte(0);
    soft_i2c_stop();
    return data;
}

/*!
    \brief      写入单个寄存器
    \param[in]  i2c_periph: I2C 外设
    \param[in]  dev_addr: 设备 7 位地址
    \param[in]  reg: 寄存器地址
    \param[in]  data: 待写入数据
    \retval     无
*/
void i2c_reg_write(uint32_t i2c_periph, uint8_t dev_addr, uint8_t reg,
                   uint8_t data) {
    soft_i2c_start();
    soft_i2c_send_byte((dev_addr << 1) | 0);
    soft_i2c_send_byte(reg);
    soft_i2c_send_byte(data);
    soft_i2c_stop();
}

/*!
    \brief      连续批量读取多字节
    \param[in]  i2c_periph: I2C 外设
    \param[in]  dev_addr: 设备 7 位地址
    \param[in]  reg: 起始寄存器地址
    \param[out] buf: 接收缓冲区
    \param[in]  len: 要读取的字节数
    \retval     无
*/
void i2c_reg_read_multi(uint32_t i2c_periph, uint8_t dev_addr, uint8_t reg,
                        uint8_t *buf, uint16_t len) {
    soft_i2c_start();
    soft_i2c_send_byte((dev_addr << 1) | 0);
    soft_i2c_send_byte(reg);
    soft_i2c_start();
    soft_i2c_send_byte((dev_addr << 1) | 1);

    for (uint16_t i = 0; i < len; i++) {
        buf[i] = soft_i2c_read_byte((i < len - 1) ? 1 : 0);
    }

    soft_i2c_stop();
}
