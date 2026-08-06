#include "gd32f3x0.h"
#include "systick.h"
#include "hard_i2c.h"
#include <stdio.h>

#define I2C_TIME_OUT I2C_TIMEOUT

/*!
    \brief      配置硬件I2C1的GPIO引脚
    \param[in]  无
    \param[out] 无
    \retval     无
*/
static void hard_i2c_gpio_config(void)
{
    /* 使能GPIO时钟 */
    rcu_periph_clock_enable(RCU_GPIOA);
    /* 使能I2C1时钟 */
    rcu_periph_clock_enable(RCU_I2C1);

    /* 连接PA0到I2C1_SCL */
    gpio_af_set(HARD_I2C_PORT, GPIO_AF_1, HARD_I2C_SCL_PIN);
    /* 连接PA1到I2C1_SDA */
    gpio_af_set(HARD_I2C_PORT, GPIO_AF_1, HARD_I2C_SDA_PIN);

    /* 配置GPIO为复用功能、开漏输出、上拉 */
    gpio_mode_set(HARD_I2C_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, 
                  HARD_I2C_SCL_PIN | HARD_I2C_SDA_PIN);
    gpio_output_options_set(HARD_I2C_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,
                            HARD_I2C_SCL_PIN | HARD_I2C_SDA_PIN);
}

/*!
    \brief      配置硬件I2C1接口
    \param[in]  无
    \param[out] 无
    \retval     无
*/
static void hard_i2c_config(void)
{
    /* 配置I2C时钟速度 */
    i2c_clock_config(HARD_I2C_PERIPH, HARD_I2C_SPEED, I2C_DTCY_2);
    /* 配置I2C地址模式 */
    i2c_mode_addr_config(HARD_I2C_PERIPH, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, 0xA0);
    /* 使能I2C */
    i2c_enable(HARD_I2C_PERIPH);
    /* 使能ACK */
    i2c_ack_config(HARD_I2C_PERIPH, I2C_ACK_ENABLE);
}

/*!
    \brief      初始化硬件I2C1
    \param[in]  无
    \param[out] 无
    \retval     无
*/
void hard_i2c_init(void)
{
    hard_i2c_gpio_config();
    hard_i2c_config();
}

/*!
    \brief      写入单个寄存器
    \param[in]  i2c_periph: I2C外设（保留参数为兼容）
    \param[in]  dev_addr: 设备7位地址
    \param[in]  reg: 寄存器地址
    \param[in]  data: 待写入数据
    \retval     无
*/
void i2c_reg_write(uint32_t i2c_periph, uint8_t dev_addr, uint8_t reg, uint8_t data)
{
    uint8_t state = I2C_START;
    uint16_t timeout = 0;
    uint8_t i2c_timeout_flag = 0;

    /* 使能ACK */
    i2c_ack_config(HARD_I2C_PERIPH, I2C_ACK_ENABLE);

    while(!i2c_timeout_flag) {
        switch(state) {
        case I2C_START:
            /* 等待总线空闲 */
            while(i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_I2CBSY) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }

            if(timeout < I2C_TIME_OUT) {
                i2c_start_on_bus(HARD_I2C_PERIPH);
                timeout = 0;
                state = I2C_SEND_ADDRESS;
            } else {
                timeout = 0;
                state = I2C_START;
                printf("I2C bus is busy in WRITE!\n");
            }
            break;

        case I2C_SEND_ADDRESS:
            /* 等待START发送完成 */
            while((!i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_SBSEND)) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }

            if(timeout < I2C_TIME_OUT) {
                i2c_master_addressing(HARD_I2C_PERIPH, dev_addr, I2C_TRANSMITTER);
                timeout = 0;
                state = I2C_CLEAR_ADDRESS_FLAG;
            } else {
                timeout = 0;
                state = I2C_START;
                printf("I2C send start timeout in WRITE!\n");
            }
            break;

        case I2C_CLEAR_ADDRESS_FLAG:
            /* 等待地址发送完成 */
            while((!i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_ADDSEND)) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }

            if(timeout < I2C_TIME_OUT) {
                i2c_flag_clear(HARD_I2C_PERIPH, I2C_FLAG_ADDSEND);
                timeout = 0;
                state = I2C_TRANSMIT_DATA;
            } else {
                timeout = 0;
                state = I2C_START;
                printf("I2C clear address flag timeout in WRITE!\n");
            }
            break;

        case I2C_TRANSMIT_DATA:
            /* 等待发送缓冲区空 */
            while((!i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_TBE)) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }

            if(timeout < I2C_TIME_OUT) {
                i2c_data_transmit(HARD_I2C_PERIPH, reg);
                timeout = 0;
            } else {
                timeout = 0;
                state = I2C_START;
                printf("I2C transmit reg timeout in WRITE!\n");
                return;
            }

            /* 等待字节传输完成 */
            while((!i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_BTC)) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }

            if(timeout < I2C_TIME_OUT) {
                timeout = 0;
            } else {
                timeout = 0;
                state = I2C_START;
                printf("I2C transmit reg BTC timeout in WRITE!\n");
                return;
            }

            /* 发送数据 */
            i2c_data_transmit(HARD_I2C_PERIPH, data);

            /* 等待字节传输完成 */
            while((!i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_BTC)) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }

            if(timeout < I2C_TIME_OUT) {
                state = I2C_STOP;
                timeout = 0;
            } else {
                timeout = 0;
                state = I2C_START;
                printf("I2C transmit data BTC timeout in WRITE!\n");
                return;
            }
            break;

        case I2C_STOP:
            /* 发送STOP信号 */
            i2c_stop_on_bus(HARD_I2C_PERIPH);
            while((I2C_CTL0(HARD_I2C_PERIPH) & I2C_CTL0_STOP) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }

            if(timeout < I2C_TIME_OUT) {
                timeout = 0;
                state = I2C_END;
                i2c_timeout_flag = 1;
            } else {
                timeout = 0;
                state = I2C_START;
                printf("I2C send stop timeout in WRITE!\n");
            }
            break;

        default:
            state = I2C_START;
            i2c_timeout_flag = 1;
            timeout = 0;
            break;
        }
    }
}

/*!
    \brief      读取单个寄存器
    \param[in]  i2c_periph: I2C外设（保留参数为兼容）
    \param[in]  dev_addr: 设备7位地址
    \param[in]  reg: 寄存器地址
    \retval     读取到的寄存器值
*/
uint8_t i2c_reg_read(uint32_t i2c_periph, uint8_t dev_addr, uint8_t reg)
{
    uint8_t state = I2C_START;
    uint8_t read_cycle = 0;
    uint16_t timeout = 0;
    uint8_t i2c_timeout_flag = 0;
    uint8_t data = 0;

    while(!i2c_timeout_flag) {
        switch(state) {
        case I2C_START:
            if(read_cycle == 0) {
                /* 等待总线空闲 */
                while(i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_I2CBSY) && (timeout < I2C_TIME_OUT)) {
                    timeout++;
                }

                if(timeout < I2C_TIME_OUT) {
                    i2c_start_on_bus(HARD_I2C_PERIPH);
                    timeout = 0;
                    state = I2C_SEND_ADDRESS;
                } else {
                    timeout = 0;
                    state = I2C_START;
                    printf("I2C bus is busy in READ!\n");
                    return 0;
                }
            } else {
                i2c_start_on_bus(HARD_I2C_PERIPH);
                timeout = 0;
                state = I2C_SEND_ADDRESS;
            }
            break;

        case I2C_SEND_ADDRESS:
            /* 等待START发送完成 */
            while((!i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_SBSEND)) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }

            if(timeout < I2C_TIME_OUT) {
                if(read_cycle == 0) {
                    i2c_master_addressing(HARD_I2C_PERIPH, dev_addr, I2C_TRANSMITTER);
                } else {
                    i2c_master_addressing(HARD_I2C_PERIPH, dev_addr, I2C_RECEIVER);
                    i2c_ack_config(HARD_I2C_PERIPH, I2C_ACK_DISABLE);
                }
                timeout = 0;
                state = I2C_CLEAR_ADDRESS_FLAG;
            } else {
                timeout = 0;
                state = I2C_START;
                read_cycle = 0;
                printf("I2C send start timeout in READ!\n");
                return 0;
            }
            break;

        case I2C_CLEAR_ADDRESS_FLAG:
            /* 等待地址发送完成 */
            while((!i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_ADDSEND)) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }

            if(timeout < I2C_TIME_OUT) {
                i2c_flag_clear(HARD_I2C_PERIPH, I2C_FLAG_ADDSEND);
                timeout = 0;
                state = I2C_TRANSMIT_DATA;
            } else {
                timeout = 0;
                state = I2C_START;
                read_cycle = 0;
                printf("I2C clear address flag timeout in READ!\n");
                return 0;
            }
            break;

        case I2C_TRANSMIT_DATA:
            if(read_cycle == 0) {
                /* 等待发送缓冲区空 */
                while((!i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_TBE)) && (timeout < I2C_TIME_OUT)) {
                    timeout++;
                }

                if(timeout < I2C_TIME_OUT) {
                    i2c_data_transmit(HARD_I2C_PERIPH, reg);
                    timeout = 0;
                } else {
                    timeout = 0;
                    state = I2C_START;
                    read_cycle = 0;
                    printf("I2C transmit reg timeout in READ!\n");
                    return 0;
                }

                /* 等待字节传输完成 */
                while((!i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_BTC)) && (timeout < I2C_TIME_OUT)) {
                    timeout++;
                }

                if(timeout < I2C_TIME_OUT) {
                    timeout = 0;
                    state = I2C_START;
                    read_cycle++;
                } else {
                    timeout = 0;
                    state = I2C_START;
                    read_cycle = 0;
                    printf("I2C BTC timeout in READ!\n");
                    return 0;
                }
            } else {
                /* 发送STOP */
                i2c_stop_on_bus(HARD_I2C_PERIPH);
                state = I2C_RECEIVE_DATA;
            }
            break;

        case I2C_RECEIVE_DATA:
            /* 等待接收数据非空 */
            while((!i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_RBNE)) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }

            if(timeout < I2C_TIME_OUT) {
                data = i2c_data_receive(HARD_I2C_PERIPH);
                timeout = 0;
                state = I2C_STOP;
            } else {
                timeout = 0;
                state = I2C_START;
                printf("I2C receive data timeout in READ!\n");
                return 0;
            }
            break;

        case I2C_STOP:
            /* 等待STOP发送完成 */
            while((I2C_CTL0(HARD_I2C_PERIPH) & I2C_CTL0_STOP) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }

            if(timeout < I2C_TIME_OUT) {
                timeout = 0;
                state = I2C_END;
                i2c_timeout_flag = 1;
            } else {
                timeout = 0;
                state = I2C_START;
                printf("I2C stop timeout in READ!\n");
            }
            break;

        default:
            state = I2C_START;
            i2c_timeout_flag = 1;
            timeout = 0;
            break;
        }
    }

    return data;
}

/*!
    \brief      连续批量读取多字节
    \param[in]  i2c_periph: I2C外设（保留参数为兼容）
    \param[in]  dev_addr: 设备7位地址
    \param[in]  reg: 起始寄存器地址
    \param[out] buf: 接收缓冲区
    \param[in]  len: 要读取的字节数
    \retval     无
*/
void i2c_reg_read_multi(uint32_t i2c_periph, uint8_t dev_addr, uint8_t reg,
                        uint8_t *buf, uint16_t len)
{
    uint8_t state = I2C_START;
    uint8_t read_cycle = 0;
    uint16_t timeout = 0;
    uint8_t i2c_timeout_flag = 0;
    uint16_t index = 0;

    while(!i2c_timeout_flag) {
        switch(state) {
        case I2C_START:
            if(read_cycle == 0) {
                /* 等待总线空闲 */
                while(i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_I2CBSY) && (timeout < I2C_TIME_OUT)) {
                    timeout++;
                }

                if(timeout < I2C_TIME_OUT) {
                    if(len == 2) {
                        i2c_ackpos_config(HARD_I2C_PERIPH, I2C_ACKPOS_NEXT);
                    }
                    i2c_start_on_bus(HARD_I2C_PERIPH);
                    timeout = 0;
                    state = I2C_SEND_ADDRESS;
                } else {
                    timeout = 0;
                    state = I2C_START;
                    printf("I2C bus is busy in READ_MULTI!\n");
                    return;
                }
            } else {
                i2c_start_on_bus(HARD_I2C_PERIPH);
                timeout = 0;
                state = I2C_SEND_ADDRESS;
            }
            break;

        case I2C_SEND_ADDRESS:
            /* 等待START发送完成 */
            while((!i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_SBSEND)) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }

            if(timeout < I2C_TIME_OUT) {
                if(read_cycle == 0) {
                    i2c_master_addressing(HARD_I2C_PERIPH, dev_addr, I2C_TRANSMITTER);
                } else {
                    i2c_master_addressing(HARD_I2C_PERIPH, dev_addr, I2C_RECEIVER);
                    if(len < 3) {
                        i2c_ack_config(HARD_I2C_PERIPH, I2C_ACK_DISABLE);
                    } else {
                        i2c_ack_config(HARD_I2C_PERIPH, I2C_ACK_ENABLE);
                    }
                }
                timeout = 0;
                state = I2C_CLEAR_ADDRESS_FLAG;
            } else {
                timeout = 0;
                state = I2C_START;
                read_cycle = 0;
                printf("I2C send start timeout in READ_MULTI!\n");
                return;
            }
            break;

        case I2C_CLEAR_ADDRESS_FLAG:
            /* 等待地址发送完成 */
            while((!i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_ADDSEND)) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }

            if(timeout < I2C_TIME_OUT) {
                i2c_flag_clear(HARD_I2C_PERIPH, I2C_FLAG_ADDSEND);
                if((read_cycle == 1) && (len == 1)) {
                    i2c_stop_on_bus(HARD_I2C_PERIPH);
                }
                timeout = 0;
                state = I2C_TRANSMIT_DATA;
            } else {
                timeout = 0;
                state = I2C_START;
                read_cycle = 0;
                printf("I2C clear address flag timeout in READ_MULTI!\n");
                return;
            }
            break;

        case I2C_TRANSMIT_DATA:
            if(read_cycle == 0) {
                /* 等待发送缓冲区空 */
                while((!i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_TBE)) && (timeout < I2C_TIME_OUT)) {
                    timeout++;
                }

                if(timeout < I2C_TIME_OUT) {
                    i2c_data_transmit(HARD_I2C_PERIPH, reg);
                    timeout = 0;
                } else {
                    timeout = 0;
                    state = I2C_START;
                    read_cycle = 0;
                    printf("I2C transmit reg timeout in READ_MULTI!\n");
                    return;
                }

                /* 等待字节传输完成 */
                while((!i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_BTC)) && (timeout < I2C_TIME_OUT)) {
                    timeout++;
                }

                if(timeout < I2C_TIME_OUT) {
                    timeout = 0;
                    state = I2C_START;
                    read_cycle++;
                } else {
                    timeout = 0;
                    state = I2C_START;
                    read_cycle = 0;
                    printf("I2C BTC timeout in READ_MULTI!\n");
                    return;
                }
            } else {
                state = I2C_RECEIVE_DATA;
            }
            break;

        case I2C_RECEIVE_DATA:
            while(index < len) {
                if(index == (len - 2)) {
                    if(len >= 3) {
                        i2c_ackpos_config(HARD_I2C_PERIPH, I2C_ACKPOS_NEXT);
                        i2c_ack_config(HARD_I2C_PERIPH, I2C_ACK_DISABLE);
                    }
                }

                if(index == (len - 1)) {
                    i2c_stop_on_bus(HARD_I2C_PERIPH);
                }

                /* 等待接收数据非空 */
                while((!i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_RBNE)) && (timeout < I2C_TIME_OUT)) {
                    timeout++;
                }

                if(timeout < I2C_TIME_OUT) {
                    buf[index++] = i2c_data_receive(HARD_I2C_PERIPH);
                    timeout = 0;
                } else {
                    timeout = 0;
                    state = I2C_START;
                    printf("I2C receive data timeout in READ_MULTI!\n");
                    return;
                }
            }
            state = I2C_STOP;
            break;

        case I2C_STOP:
            /* 等待STOP发送完成 */
            while((I2C_CTL0(HARD_I2C_PERIPH) & I2C_CTL0_STOP) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }

            if(timeout < I2C_TIME_OUT) {
                timeout = 0;
                state = I2C_END;
                i2c_timeout_flag = 1;
            } else {
                timeout = 0;
                state = I2C_START;
                printf("I2C stop timeout in READ_MULTI!\n");
            }
            break;

        default:
            state = I2C_START;
            i2c_timeout_flag = 1;
            timeout = 0;
            break;
        }
    }
}

/*!
    \brief      探测I2C设备是否在线
    \param[in]  dev_addr: 设备7位地址
    \retval     0 = 设备在线（收到ACK）, 1 = 设备离线（收到NACK）
*/
uint8_t hard_i2c_probe(uint8_t dev_addr)
{
    uint8_t state = I2C_START;
    uint16_t timeout = 0;
    uint8_t i2c_timeout_flag = 0;
    uint8_t result = 1; /* 默认离线 */

    while(!i2c_timeout_flag) {
        switch(state) {
        case I2C_START:
            /* 等待总线空闲 */
            while(i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_I2CBSY) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }

            if(timeout < I2C_TIME_OUT) {
                i2c_start_on_bus(HARD_I2C_PERIPH);
                timeout = 0;
                state = I2C_SEND_ADDRESS;
            } else {
                timeout = 0;
                state = I2C_START;
                return 1;
            }
            break;

        case I2C_SEND_ADDRESS:
            /* 等待START发送完成 */
            while((!i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_SBSEND)) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }

            if(timeout < I2C_TIME_OUT) {
                i2c_master_addressing(HARD_I2C_PERIPH, dev_addr, I2C_TRANSMITTER);
                timeout = 0;
                state = I2C_CLEAR_ADDRESS_FLAG;
            } else {
                timeout = 0;
                state = I2C_START;
                return 1;
            }
            break;

        case I2C_CLEAR_ADDRESS_FLAG:
            /* 等待地址发送完成 */
            while((!i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_ADDSEND)) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }

            if(timeout < I2C_TIME_OUT) {
                i2c_flag_clear(HARD_I2C_PERIPH, I2C_FLAG_ADDSEND);
                timeout = 0;
                state = I2C_STOP;
                result = 0; /* 在线 */
            } else {
                timeout = 0;
                state = I2C_STOP;
                result = 1; /* 离线 */
            }
            break;

        case I2C_STOP:
            /* 发送STOP信号 */
            i2c_stop_on_bus(HARD_I2C_PERIPH);
            while((I2C_CTL0(HARD_I2C_PERIPH) & I2C_CTL0_STOP) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }

            timeout = 0;
            i2c_timeout_flag = 1;
            break;

        default:
            state = I2C_START;
            i2c_timeout_flag = 1;
            timeout = 0;
            break;
        }
    }

    return result;
}