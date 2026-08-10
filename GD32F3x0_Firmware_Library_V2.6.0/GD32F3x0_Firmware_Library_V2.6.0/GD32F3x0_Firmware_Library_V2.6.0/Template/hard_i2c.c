#include "gd32f3x0.h"
#include "systick.h"
#include "hard_i2c.h"
#include <stdio.h>

/* ============ 调试打印开关 ============
   1 = 输出详细I2C日志（调试用，UART占用较多时间）
   0 = 关闭本文件所有I2C调试打印（正式运行） */
#define I2C_DEBUG_ENABLE 0

#if !I2C_DEBUG_ENABLE
#define printf(...)  /* 编译期关闭本文件的调试printf */
#endif

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

    /* 连接PA0到I2C1_SCL（GD32F330数据手册：PA0的I2C1_SCL是AF4，不是AF1） */
    gpio_af_set(HARD_I2C_PORT, GPIO_AF_4, HARD_I2C_SCL_PIN);
    /* 连接PA1到I2C1_SDA（GD32F330数据手册：PA1的I2C1_SDA是AF4） */
    gpio_af_set(HARD_I2C_PORT, GPIO_AF_4, HARD_I2C_SDA_PIN);

    /* 配置GPIO为复用功能、开漏输出、上拉 */
    gpio_mode_set(HARD_I2C_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, 
                  HARD_I2C_SCL_PIN | HARD_I2C_SDA_PIN);
    gpio_output_options_set(HARD_I2C_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,
                            HARD_I2C_SCL_PIN | HARD_I2C_SDA_PIN);
}

/*!
    \brief      复位I2C总线（通过GPIO手动释放）
    \param[in]  无
    \param[out] 无
    \retval     无
*/
static void hard_i2c_bus_reset(void)
{
    uint16_t i;
    
    printf("I2C bus reset starting...\n");
    printf("  Before reset: BUSY=%d, SCL=%d, SDA=%d\n", 
           i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_I2CBSY),
           gpio_input_bit_get(HARD_I2C_PORT, HARD_I2C_SCL_PIN),
           gpio_input_bit_get(HARD_I2C_PORT, HARD_I2C_SDA_PIN));
    
    /* 先禁用I2C外设 */
    i2c_disable(HARD_I2C_PERIPH);
    
    /* 将GPIO切换为普通输出模式，使用开漏输出（I2C要求）*/
    gpio_mode_set(HARD_I2C_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, 
                  HARD_I2C_SCL_PIN | HARD_I2C_SDA_PIN);
    gpio_output_options_set(HARD_I2C_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,
                            HARD_I2C_SCL_PIN | HARD_I2C_SDA_PIN);
    
    /* 先检查引脚状态 */
    printf("  After GPIO config: SCL=%d, SDA=%d\n",
           gpio_input_bit_get(HARD_I2C_PORT, HARD_I2C_SCL_PIN),
           gpio_input_bit_get(HARD_I2C_PORT, HARD_I2C_SDA_PIN));
    
    /* 拉低SCL */
    gpio_bit_write(HARD_I2C_PORT, HARD_I2C_SCL_PIN, RESET);
    delay_1ms(1);
    
    /* 拉低SDA */
    gpio_bit_write(HARD_I2C_PORT, HARD_I2C_SDA_PIN, RESET);
    delay_1ms(1);
    
    /* 释放SCL（拉高）*/
    gpio_bit_write(HARD_I2C_PORT, HARD_I2C_SCL_PIN, SET);
    delay_1ms(1);
    
    /* 释放SDA（拉高）*/
    gpio_bit_write(HARD_I2C_PORT, HARD_I2C_SDA_PIN, SET);
    delay_1ms(1);
    
    printf("  After release: SCL=%d, SDA=%d\n",
           gpio_input_bit_get(HARD_I2C_PORT, HARD_I2C_SCL_PIN),
           gpio_input_bit_get(HARD_I2C_PORT, HARD_I2C_SDA_PIN));
    
    /* 产生9个时钟脉冲，释放可能被锁住的从设备 */
    for(i = 0; i < 9; i++) {
        gpio_bit_write(HARD_I2C_PORT, HARD_I2C_SCL_PIN, RESET);
        delay_1ms(1);
        gpio_bit_write(HARD_I2C_PORT, HARD_I2C_SCL_PIN, SET);
        delay_1ms(1);
    }
    
    /* 产生STOP信号：SDA先低，然后SCL高，最后SDA高 */
    gpio_bit_write(HARD_I2C_PORT, HARD_I2C_SDA_PIN, RESET);
    delay_1ms(1);
    gpio_bit_write(HARD_I2C_PORT, HARD_I2C_SCL_PIN, SET);
    delay_1ms(1);
    gpio_bit_write(HARD_I2C_PORT, HARD_I2C_SDA_PIN, SET);
    delay_1ms(1);
    
    printf("  After clock pulses: SCL=%d, SDA=%d\n",
           gpio_input_bit_get(HARD_I2C_PORT, HARD_I2C_SCL_PIN),
           gpio_input_bit_get(HARD_I2C_PORT, HARD_I2C_SDA_PIN));
    
    /* 重新配置GPIO为复用开漏模式（I2C1在PA0/PA1上是AF4） */
    gpio_af_set(HARD_I2C_PORT, GPIO_AF_4, HARD_I2C_SCL_PIN);
    gpio_af_set(HARD_I2C_PORT, GPIO_AF_4, HARD_I2C_SDA_PIN);
    gpio_mode_set(HARD_I2C_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, 
                  HARD_I2C_SCL_PIN | HARD_I2C_SDA_PIN);
    gpio_output_options_set(HARD_I2C_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,
                            HARD_I2C_SCL_PIN | HARD_I2C_SDA_PIN);
    
    /* 彻底复位I2C外设 - 使用RCU外设复位 */
    i2c_disable(HARD_I2C_PERIPH);
    
    /* 对I2C1进行RCU复位 */
    rcu_periph_reset_enable(RCU_I2C1RST);
    delay_1ms(1);
    rcu_periph_reset_disable(RCU_I2C1RST);
    delay_1ms(1);
    
    /* 清除所有标志位 */
    i2c_flag_clear(HARD_I2C_PERIPH, I2C_FLAG_ADDSEND);
    i2c_flag_clear(HARD_I2C_PERIPH, I2C_FLAG_RBNE);
    i2c_flag_clear(HARD_I2C_PERIPH, I2C_FLAG_TBE);
    i2c_flag_clear(HARD_I2C_PERIPH, I2C_FLAG_BTC);
    
    /* 重新配置I2C外设 */
    i2c_clock_config(HARD_I2C_PERIPH, HARD_I2C_SPEED, I2C_DTCY_2);
    i2c_mode_addr_config(HARD_I2C_PERIPH, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, 0xA0);
    i2c_ack_config(HARD_I2C_PERIPH, I2C_ACK_ENABLE);
    
    /* 使能I2C前等待总线空闲 */
    uint16_t wait_timeout = 0;
    while(i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_I2CBSY) && (wait_timeout < 1000)) {
        wait_timeout++;
    }
    
    i2c_enable(HARD_I2C_PERIPH);
    
    printf("  After I2C re-init: BUSY=%d, SCL=%d, SDA=%d\n",
           i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_I2CBSY),
           gpio_input_bit_get(HARD_I2C_PORT, HARD_I2C_SCL_PIN),
           gpio_input_bit_get(HARD_I2C_PORT, HARD_I2C_SDA_PIN));
    printf("  CTL0=0x%04X, STAT0=0x%04X\n", 
           I2C_CTL0(HARD_I2C_PERIPH), I2C_STAT0(HARD_I2C_PERIPH));
    
    printf("I2C bus reset done!\n");
}

/*!
    \brief      配置硬件I2C1接口
    \param[in]  无
    \param[out] 无
    \retval     无
*/
static void hard_i2c_config(void)
{
    /* 先禁用I2C（如果之前已使能） */
    i2c_disable(HARD_I2C_PERIPH);
    
    /* 配置I2C时钟速度 */
    i2c_clock_config(HARD_I2C_PERIPH, HARD_I2C_SPEED, I2C_DTCY_2);
    /* 配置I2C地址模式 */
    i2c_mode_addr_config(HARD_I2C_PERIPH, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, 0xA0);
    /* 使能ACK */
    i2c_ack_config(HARD_I2C_PERIPH, I2C_ACK_ENABLE);
    /* 最后使能I2C */
    i2c_enable(HARD_I2C_PERIPH);
}

/*!
    \brief      初始化硬件I2C1
    \param[in]  无
    \param[out] 无
    \retval     无
*/
void hard_i2c_init(void)
{
    /* 打印GPIO引脚状态 */
    printf("Before I2C init:\n");
    printf("  SCL pin state: %d\n", gpio_input_bit_get(HARD_I2C_PORT, HARD_I2C_SCL_PIN));
    printf("  SDA pin state: %d\n", gpio_input_bit_get(HARD_I2C_PORT, HARD_I2C_SDA_PIN));
    
    hard_i2c_gpio_config();
    hard_i2c_config();

    /* 打印I2C寄存器状态 */
    printf("I2C CTL0: 0x%04X\n", I2C_CTL0(HARD_I2C_PERIPH));
    printf("I2C CTL1: 0x%04X\n", I2C_CTL1(HARD_I2C_PERIPH));
    printf("I2C CKCFG: 0x%04X\n", I2C_CKCFG(HARD_I2C_PERIPH));
    printf("I2C RT: 0x%04X\n", I2C_RT(HARD_I2C_PERIPH));
    printf("I2C STAT0: 0x%04X\n", I2C_STAT0(HARD_I2C_PERIPH));
    printf("I2C STAT1: 0x%04X\n", I2C_STAT1(HARD_I2C_PERIPH));
    printf("GPIO AF: 0x%08X\n", GPIO_AFSEL0(HARD_I2C_PORT));

    /* 等待I2C总线空闲 */
    uint16_t timeout = 0;
    while(i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_I2CBSY) && (timeout < I2C_TIME_OUT)) {
        timeout++;
    }
    
    if(timeout >= I2C_TIME_OUT) {
        printf("I2C init: bus is busy, doing bus reset...\n");
        hard_i2c_bus_reset();
    } else {
        printf("I2C init: OK\n");
    }
    
    /* 再次检查总线状态 */
    printf("After I2C init:\n");
    printf("  Bus busy flag: %d\n", i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_I2CBSY));
    printf("  SCL pin state: %d\n", gpio_input_bit_get(HARD_I2C_PORT, HARD_I2C_SCL_PIN));
    printf("  SDA pin state: %d\n", gpio_input_bit_get(HARD_I2C_PORT, HARD_I2C_SDA_PIN));
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

    /* GD32硬件I2C需要8位地址（7位地址左移1位），与soft_i2c保持一致 */
    uint8_t addr_8bit = dev_addr << 1;

    printf("WRITE: dev_addr=0x%02X (8bit=0x%02X), reg=0x%02X, data=0x%02X\n", dev_addr, addr_8bit, reg, data);
    printf("  Before START: BUSY=%d\n", i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_I2CBSY));

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
                hard_i2c_bus_reset();
            }
            break;

        case I2C_SEND_ADDRESS:
            /* 等待START发送完成 */
            while((!i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_SBSEND)) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }

            if(timeout < I2C_TIME_OUT) {
                /* 先写地址再打印：避免START到写地址之间的printf延时把SCL长时间拉低，
                   某些从机(如ICM42670)会因此NACK */
                i2c_master_addressing(HARD_I2C_PERIPH, addr_8bit, I2C_TRANSMITTER);
                printf("  START sent OK, sending address 0x%02X (W)\n", addr_8bit);
                timeout = 0;
                state = I2C_CLEAR_ADDRESS_FLAG;
            } else {
                timeout = 0;
                state = I2C_START;
                printf("I2C send start timeout in WRITE!\n");
                printf("  STAT0=0x%04X, STAT1=0x%04X\n", I2C_STAT0(HARD_I2C_PERIPH), I2C_STAT1(HARD_I2C_PERIPH));
                hard_i2c_bus_reset();
            }
            break;

        case I2C_CLEAR_ADDRESS_FLAG:
            /* 等待地址发送完成 */
            while((!i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_ADDSEND)) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }

            if(timeout < I2C_TIME_OUT) {
                printf("  Address sent OK, clearing ADDSEND\n");
                i2c_flag_clear(HARD_I2C_PERIPH, I2C_FLAG_ADDSEND);
                timeout = 0;
                state = I2C_TRANSMIT_DATA;
            } else {
                timeout = 0;
                state = I2C_START;
                printf("I2C clear address flag timeout in WRITE!\n");
                printf("  STAT0=0x%04X, STAT1=0x%04X\n", I2C_STAT0(HARD_I2C_PERIPH), I2C_STAT1(HARD_I2C_PERIPH));
                hard_i2c_bus_reset();
            }
            break;

        case I2C_TRANSMIT_DATA:
            /* 等待发送缓冲区空 */
            while((!i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_TBE)) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }

            if(timeout < I2C_TIME_OUT) {
                printf("  TBE OK, sending reg=0x%02X\n", reg);
                i2c_data_transmit(HARD_I2C_PERIPH, reg);
                timeout = 0;
            } else {
                timeout = 0;
                state = I2C_START;
                printf("I2C transmit reg timeout in WRITE!\n");
                printf("  STAT0=0x%04X, STAT1=0x%04X\n", I2C_STAT0(HARD_I2C_PERIPH), I2C_STAT1(HARD_I2C_PERIPH));
                hard_i2c_bus_reset();
                return;
            }

            /* 等待字节传输完成 */
            while((!i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_BTC)) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }

            if(timeout < I2C_TIME_OUT) {
                printf("  Reg BTC OK\n");
                timeout = 0;
            } else {
                timeout = 0;
                state = I2C_START;
                printf("I2C transmit reg BTC timeout in WRITE!\n");
                printf("  STAT0=0x%04X, STAT1=0x%04X\n", I2C_STAT0(HARD_I2C_PERIPH), I2C_STAT1(HARD_I2C_PERIPH));
                hard_i2c_bus_reset();
                return;
            }

            /* 发送数据 */
            i2c_data_transmit(HARD_I2C_PERIPH, data);

            /* 等待字节传输完成 */
            while((!i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_BTC)) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }

            if(timeout < I2C_TIME_OUT) {
                printf("  Data BTC OK\n");
                state = I2C_STOP;
                timeout = 0;
            } else {
                timeout = 0;
                state = I2C_START;
                printf("I2C transmit data BTC timeout in WRITE!\n");
                printf("  STAT0=0x%04X, STAT1=0x%04X\n", I2C_STAT0(HARD_I2C_PERIPH), I2C_STAT1(HARD_I2C_PERIPH));
                hard_i2c_bus_reset();
                return;
            }
            break;

        case I2C_STOP:
            /* 发送STOP信号 */
            printf("  Sending STOP\n");
            i2c_stop_on_bus(HARD_I2C_PERIPH);
            while((I2C_CTL0(HARD_I2C_PERIPH) & I2C_CTL0_STOP) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }

            if(timeout < I2C_TIME_OUT) {
                printf("  WRITE done!\n");
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

    /* GD32硬件I2C需要8位地址（7位地址左移1位），与soft_i2c保持一致 */
    uint8_t addr_8bit = dev_addr << 1;

    printf("READ: dev_addr=0x%02X (8bit=0x%02X), reg=0x%02X\n", dev_addr, addr_8bit, reg);

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
                    hard_i2c_bus_reset();
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
                    /* 先写地址再打印，避免START到写地址之间的延时导致从机NACK */
                    i2c_master_addressing(HARD_I2C_PERIPH, addr_8bit, I2C_TRANSMITTER);
                    printf("  START sent, sending write addr 0x%02X\n", addr_8bit);
                } else {
                    i2c_master_addressing(HARD_I2C_PERIPH, addr_8bit, I2C_RECEIVER);
                    i2c_ack_config(HARD_I2C_PERIPH, I2C_ACK_DISABLE);
                    printf("  Re-START sent, sending read addr 0x%02X\n", addr_8bit | 0x01);
                }
                timeout = 0;
                state = I2C_CLEAR_ADDRESS_FLAG;
            } else {
                timeout = 0;
                state = I2C_START;
                read_cycle = 0;
                printf("I2C send start timeout in READ!\n");
                hard_i2c_bus_reset();
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
                hard_i2c_bus_reset();
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
                    hard_i2c_bus_reset();
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
                    hard_i2c_bus_reset();
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
                hard_i2c_bus_reset();
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
                hard_i2c_bus_reset();
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

    /* GD32硬件I2C需要8位地址（7位地址左移1位），与soft_i2c保持一致 */
    uint8_t addr_8bit = dev_addr << 1;

    printf("READ_MULTI: dev_addr=0x%02X (8bit=0x%02X), reg=0x%02X, len=%d\n", dev_addr, addr_8bit, reg, len);

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
                    hard_i2c_bus_reset();
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
                    i2c_master_addressing(HARD_I2C_PERIPH, addr_8bit, I2C_TRANSMITTER);
                } else {
                    i2c_master_addressing(HARD_I2C_PERIPH, addr_8bit, I2C_RECEIVER);
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
                hard_i2c_bus_reset();
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
                hard_i2c_bus_reset();
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
                    hard_i2c_bus_reset();
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
                    hard_i2c_bus_reset();
                    return;
                }
            } else {
                state = I2C_RECEIVE_DATA;
            }
            break;

        case I2C_RECEIVE_DATA:
            if(len == 1) {
                /* 单字节：等RBNE读1字节（STOP已在清ADDSEND时发出，与i2c_reg_read一致） */
                while((!i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_RBNE)) && (timeout < I2C_TIME_OUT)) {
                    timeout++;
                }
                if(timeout < I2C_TIME_OUT) {
                    buf[0] = i2c_data_receive(HARD_I2C_PERIPH);
                    timeout = 0;
                    state = I2C_STOP;
                } else {
                    timeout = 0;
                    state = I2C_START;
                    printf("I2C receive data timeout in READ_MULTI! len=1\n");
                    hard_i2c_bus_reset();
                    return;
                }
            } else if(len == 2) {
                /* 双字节：官方Master_receiver_two_bytes时序
                   （ACKPOS_NEXT在START阶段已设、ACK在发读地址后已关闭） */
                while((!i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_BTC)) && (timeout < I2C_TIME_OUT)) {
                    timeout++;
                }
                if(timeout < I2C_TIME_OUT) {
                    while((!i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_RBNE)) && (timeout < I2C_TIME_OUT)) {
                        timeout++;
                    }
                }
                if(timeout < I2C_TIME_OUT) {
                    buf[0] = i2c_data_receive(HARD_I2C_PERIPH);
                    timeout = 0;
                } else {
                    timeout = 0;
                    state = I2C_START;
                    printf("I2C receive data timeout in READ_MULTI! len=2\n");
                    hard_i2c_bus_reset();
                    return;
                }
                while((!i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_RBNE)) && (timeout < I2C_TIME_OUT)) {
                    timeout++;
                }
                if(timeout < I2C_TIME_OUT) {
                    buf[1] = i2c_data_receive(HARD_I2C_PERIPH);
                    timeout = 0;
                    i2c_stop_on_bus(HARD_I2C_PERIPH);
                    state = I2C_STOP;
                } else {
                    timeout = 0;
                    state = I2C_START;
                    printf("I2C receive data timeout in READ_MULTI! len=2\n");
                    hard_i2c_bus_reset();
                    return;
                }
            } else {
                /* N>=3：官方Master_receiver(N字节)时序。
                   倒数第二个字节(index==len-2)等BTC后再关ACK，让最后一字节收到NACK；
                   STOP在全部字节收完之后才发出。 */
                while(index < len) {
                    if(index == (len - 2)) {
                        while((!i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_BTC)) && (timeout < I2C_TIME_OUT)) {
                            timeout++;
                        }
                        if(timeout >= I2C_TIME_OUT) {
                            timeout = 0;
                            state = I2C_START;
                            printf("I2C BTC timeout in READ_MULTI! index=%d\n", index);
                            hard_i2c_bus_reset();
                            return;
                        }
                        i2c_ack_config(HARD_I2C_PERIPH, I2C_ACK_DISABLE);
                        timeout = 0;
                    }

                    while((!i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_RBNE)) && (timeout < I2C_TIME_OUT)) {
                        timeout++;
                    }

                    if(timeout < I2C_TIME_OUT) {
                        buf[index++] = i2c_data_receive(HARD_I2C_PERIPH);
                        timeout = 0;
                    } else {
                        timeout = 0;
                        state = I2C_START;
                        printf("I2C receive data timeout in READ_MULTI! index=%d\n", index);
                        hard_i2c_bus_reset();
                        return;
                    }
                }
                i2c_stop_on_bus(HARD_I2C_PERIPH);
                state = I2C_STOP;
            }
            break;

        case I2C_STOP:
            /* 等待STOP发送完成 */
            while((I2C_CTL0(HARD_I2C_PERIPH) & I2C_CTL0_STOP) && (timeout < I2C_TIME_OUT)) {
                timeout++;
            }

            if(timeout < I2C_TIME_OUT) {
                /* 恢复ACK和ACKPOS，供下一次传输使用 */
                i2c_ackpos_config(HARD_I2C_PERIPH, I2C_ACKPOS_CURRENT);
                i2c_ack_config(HARD_I2C_PERIPH, I2C_ACK_ENABLE);
                timeout = 0;
                state = I2C_END;
                i2c_timeout_flag = 1;
            } else {
                timeout = 0;
                state = I2C_START;
                printf("I2C stop timeout in READ_MULTI!\n");
                hard_i2c_bus_reset();
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
    uint16_t timeout = 0;
    uint8_t result = 1; /* 默认离线 */
    volatile uint32_t stat0, stat1;

    /* GD32硬件I2C需要8位地址（7位地址左移1位），与soft_i2c保持一致 */
    uint8_t addr_8bit = dev_addr << 1;

    /* 确保总线空闲 */
    timeout = 0;
    while(i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_I2CBSY) && (timeout < I2C_TIME_OUT)) {
        timeout++;
    }
    if(timeout >= I2C_TIME_OUT) {
        printf("PROBE 0x%02X: bus busy, resetting\n", dev_addr);
        hard_i2c_bus_reset();
        return 1;
    }

    /* 使能ACK */
    i2c_ack_config(HARD_I2C_PERIPH, I2C_ACK_ENABLE);

    /* 发送START */
    i2c_start_on_bus(HARD_I2C_PERIPH);

    timeout = 0;
    while((!i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_SBSEND)) && (timeout < I2C_TIME_OUT)) {
        timeout++;
    }
    if(timeout >= I2C_TIME_OUT) {
        printf("PROBE 0x%02X: START timeout, resetting\n", dev_addr);
        hard_i2c_bus_reset();
        return 1;
    }

    /* 清除START标志 - GD32需要读STAT0 */
    stat0 = I2C_STAT0(HARD_I2C_PERIPH);

    /* 发送地址+W（先写地址再打印，避免START到写地址之间的大延时） */
    i2c_master_addressing(HARD_I2C_PERIPH, addr_8bit, I2C_TRANSMITTER);

    /* 打印START和地址发送后的寄存器状态 */
    printf("PROBE 0x%02X: After START+addr, CTL0=0x%04X, STAT0=0x%04X\n", 
           dev_addr, I2C_CTL0(HARD_I2C_PERIPH), I2C_STAT0(HARD_I2C_PERIPH));

    /* 等待 ADDSEND(有设备ACK) 或 AERR(无设备NACK)
       GD32硬件I2C在收到NACK时只置AERR、不置ADDSEND，
       因此必须同时等这两个标志，否则NACK会一直等到超时。 */
    timeout = 0;
    while((!i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_ADDSEND)) &&
          (!i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_AERR)) &&
          (timeout < I2C_TIME_OUT)) {
        timeout++;
    }

    if(i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_AERR)) {
        /* NACK - 设备离线。必须先清除AERR(及可能存在的BERR/LOSTARB)
           再发STOP，否则STOP不完成、CTL0的STOP位残留，导致下一次START卡死 */
        printf("PROBE 0x%02X: NACK (AERR)\n", dev_addr);
        i2c_flag_clear(HARD_I2C_PERIPH, I2C_FLAG_AERR);
        i2c_flag_clear(HARD_I2C_PERIPH, I2C_FLAG_BERR);
        i2c_flag_clear(HARD_I2C_PERIPH, I2C_FLAG_LOSTARB);
        result = 1;
    } else if(i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_ADDSEND)) {
        /* ACK - 设备在线，读STAT0+STAT1清除ADDSEND */
        result = 0;
        stat0 = I2C_STAT0(HARD_I2C_PERIPH);
        stat1 = I2C_STAT1(HARD_I2C_PERIPH);
    } else {
        /* 超时 - 打印现场（STAT0/STAT1/CTL0/BUSY/引脚电平）便于定位：
           - AERR 置位      -> 设备NACK（正常离线）
           - LOSTARB 置位   -> 总线冲突（多主机/外部拉低）
           - BERR 置位      -> 总线错误（意外START/STOP）
           - SCL=0 卡低     -> 时钟被拉伸卡死（上拉太弱/器件拉低）
           - SDA=0 卡低     -> 有设备把SDA拉死
           - BUSY=1         -> 传输进行中但没完成（上拉太弱导致SCL太慢） */
        printf("PROBE 0x%02X: addr timeout, STAT0=0x%04X, STAT1=0x%04X, "
               "CTL0=0x%04X, BUSY=%d, SCL=%d, SDA=%d\n",
               dev_addr,
               I2C_STAT0(HARD_I2C_PERIPH), I2C_STAT1(HARD_I2C_PERIPH),
               I2C_CTL0(HARD_I2C_PERIPH),
               i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_I2CBSY),
               gpio_input_bit_get(HARD_I2C_PORT, HARD_I2C_SCL_PIN),
               gpio_input_bit_get(HARD_I2C_PORT, HARD_I2C_SDA_PIN));
        hard_i2c_bus_reset();
        return 1;
    }

    /* 发送STOP释放总线，并确认STOP位被硬件清除 */
    i2c_stop_on_bus(HARD_I2C_PERIPH);
    timeout = 0;
    while((I2C_CTL0(HARD_I2C_PERIPH) & I2C_CTL0_STOP) && (timeout < I2C_TIME_OUT)) {
        timeout++;
    }
    if(timeout >= I2C_TIME_OUT) {
        /* STOP未完成，总线可能被卡住，做一次复位恢复 */
        printf("PROBE 0x%02X: STOP not cleared, resetting\n", dev_addr);
        hard_i2c_bus_reset();
        return 1;
    }

    /* 等待总线空闲 */
    timeout = 0;
    while(i2c_flag_get(HARD_I2C_PERIPH, I2C_FLAG_I2CBSY) && (timeout < I2C_TIME_OUT)) {
        timeout++;
    }

    return result;
}