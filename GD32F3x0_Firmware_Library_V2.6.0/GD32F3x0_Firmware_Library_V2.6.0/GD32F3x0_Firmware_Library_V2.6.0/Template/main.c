#include "main.h"
#include "communicate_protocol.h"
#include "gd32f3x0.h"
#include "stdio.h"
#include "string.h"
#include "systick.h"
#include "soft_i2c.h"

#define delay_ms(x) delay_1ms(x)

/************************ 全局 AHRS 实例 ************************/
ahrs_t ahrs;

/************************ 宏定义区域 ************************/
#define USART_RS485 USART1 // RS485上报串口

/************************ 全局变量 ************************/

#define ARRAYNUM(arr_nanme) (uint32_t)(sizeof(arr_nanme) / sizeof(*(arr_nanme)))
#define TRANSMIT_SIZE (ARRAYNUM(transmitter_buffer) - 1)

uint8_t transmitter_buffer[128] = "\n\rUSART interrupt test\n\r";
uint8_t receiver_buffer[32];
uint8_t transfersize = TRANSMIT_SIZE;
uint8_t receivesize = 32;
__IO uint8_t txcount = 0;
__IO uint16_t rxcount = 0;

/************************ USART0 DMA + 空闲中断接收 ************************/
#define USART0_RX_BUF_SIZE 256U
uint8_t usart0_rx_buffer[USART0_RX_BUF_SIZE];
volatile uint16_t usart0_rx_len = 0;
volatile uint8_t usart0_rx_flag = 0;

volatile uint8_t imu_loop_flag = 0; // 定时器中断标志

/************************ RS485串口上报函数 ************************/
/**
 * @brief 串口发送字符串
 */
void usart_send_string(char *str) {
  uint16_t len = strlen(str);
  for (uint16_t i = 0; i < len; i++) {
    while (usart_flag_get(USART_RS485, USART_FLAG_TBE) == RESET)
      ;
    usart_data_transmit(USART_RS485, str[i]);
  }
  while (usart_flag_get(USART_RS485, USART_FLAG_TC) == RESET)
    ;
}

/************************ 主循环调度函数（10ms定时调用）************************/
void imu_main_loop(uint8_t rtc_hour) {
  ahrs_update(&ahrs, rtc_hour);
}

/************************ 系统总初始化入口 ************************/
void imu_system_init(void) {
  ahrs_init(&ahrs);
  ahrs_sensor_init(&ahrs);
}

// ## 外部补充初始化说明（需要自行添加）
// 1. `delay_ms()`：基于Systick实现毫秒延时函数；
// 2. I2C0、I2C1 GPIO+外设初始化；
// 3. USART1串口初始化（RS485收发）；
// 4. 定时器10ms中断，循环调用 `imu_main_loop(rtc_hour)`；
// 5. RTC时钟读取小时，传入主循环；
// 6. 上位机下发标定指令时调用 `save_install_zero_point()` 保存安装零点。

// ## 核心业务逻辑亮点
// 1.
// **大车磁场干扰规避**：白天车流高峰自动关闭地磁航向融合，仅用陀螺积分计算绕杆旋转，不会因货车金属车体造成角度跳变误报警；
// 2.
// **三轴统一报警**：俯仰X、横滚Y、旋转Z任意一轴偏移＞10°，持续2秒稳定后上报，区分倾斜/旋转两种故障码；
// 3. **断电保存标定零点**：Flash双备份存储安装基准，设备拆装后无需重新标定；
// 4. **陀螺温漂抑制**：设备静止时自动平滑更新Z轴零偏，长期运行角度漂移小；
// 5.
// **夜间漂移修正**：夜间车流稀少，地磁缓慢修正陀螺长时间积分误差，保证长期精度。

void com_usart_init(void) {
  // /* enable GPIO clock */
  // rcu_periph_clock_enable(RCU_GPIOA);
  // /* enable USART clock */
  // rcu_periph_clock_enable(RCU_USART0);

  // /* connect port to USART TX */
  // gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_9);

  // /* connect port to USART RX */
  // gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_10);

  // /* configure USART TX as alternate function push-pull */
  // gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
  // gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ,
  // GPIO_PIN_9);

  // /* configure USART RX as alternate function push-pull */
  // gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
  // gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ,
  // GPIO_PIN_10);
  // /* USART configure */
  // usart_deinit(USART0);
  // usart_word_length_set(USART0, USART_WL_8BIT);
  // usart_stop_bit_set(USART0, USART_STB_1BIT);
  // usart_parity_config(USART0, USART_PM_NONE);
  // usart_baudrate_set(USART0, 115200U);
  // usart_receive_config(USART0, USART_RECEIVE_ENABLE);
  // usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);

  // usart_enable(USART0);

  /* enable GPIO clock */
  rcu_periph_clock_enable(RCU_GPIOA);
  /* enable USART clock */
  rcu_periph_clock_enable(RCU_USART0);

  /* connect port to USART TX */
  gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_9);

  /* connect port to USART RX */
  gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_10);

  /* configure USART TX as alternate function push-pull */
  gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
  gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_9);

  /* configure USART RX as alternate function push-pull */
  gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
  gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_10);

  /* configure USART */
  usart_deinit(USART0);
  usart_word_length_set(USART0, USART_WL_8BIT);
  usart_stop_bit_set(USART0, USART_STB_1BIT);
  usart_parity_config(USART0, USART_PM_NONE);
  usart_baudrate_set(USART0, 9600U);
  usart_receive_config(USART0, USART_RECEIVE_ENABLE);
  usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
  usart_enable(USART0);
}

/************************ USART0 DMA + 空闲中断接收配置
 * ************************/
/**
 * @brief 配置USART0 DMA接收（循环模式）+ 空闲中断
 * @note  printf用的USART0，RX用DMA_CH2，空闲中断触发后算长度
 */
void usart0_rx_dma_idle_init(void) {
  dma_parameter_struct dma_para;

  rcu_periph_clock_enable(RCU_DMA);

  dma_deinit(DMA_CH2);
  dma_struct_para_init(&dma_para);

  dma_para.direction = DMA_PERIPHERAL_TO_MEMORY;
  dma_para.memory_addr = (uint32_t)usart0_rx_buffer;
  dma_para.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
  dma_para.memory_width = DMA_MEMORY_WIDTH_8BIT;
  dma_para.number = USART0_RX_BUF_SIZE;
  dma_para.periph_addr = (uint32_t)&USART_RDATA(USART0);
  dma_para.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
  dma_para.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
  dma_para.priority = DMA_PRIORITY_HIGH;
  dma_init(DMA_CH2, &dma_para);

  dma_circulation_disable(DMA_CH2); /* 单次模式，每帧重新配置 */
  dma_memory_to_memory_disable(DMA_CH2);

  usart_flag_clear(USART0, USART_FLAG_IDLE);
  usart_interrupt_flag_clear(USART0, USART_INT_FLAG_IDLE);

  usart_dma_receive_config(USART0, USART_RECEIVE_DMA_ENABLE);
  usart_interrupt_enable(USART0, USART_INT_IDLE);

  dma_channel_enable(DMA_CH2);
  nvic_irq_enable(USART0_IRQn, 1, 0);

  /* 清除开机时 USART 可能已收到的脏数据 */
  delay_1ms(5);                         // 等 USART 线稳定
  volatile uint8_t dummy;
  for (int i = 0; i < 16; i++) {
    dummy = USART_RDATA(USART0);        // 读走残留字节
  }
  (void)dummy;
  usart_flag_clear(USART0, USART_FLAG_IDLE);
  usart_interrupt_flag_clear(USART0, USART_INT_FLAG_IDLE);
  memset(usart0_rx_buffer, 0, USART0_RX_BUF_SIZE);  // 清空 DMA 缓冲区
  usart0_rx_len = 0;
  usart0_rx_flag = 0;
  dma_transfer_number_config(DMA_CH2, USART0_RX_BUF_SIZE);  // 复位 DMA 计数器
}

/************************ USART0 DMA 发送 ************************/
volatile uint8_t usart0_tx_busy = 0;

/**
 * @brief USART0 DMA发送（USART0_TX = DMA_CH1）
 * @param buf: 待发送数据
 * @param len: 长度
 * @note  单次发送，非循环。发送期间 usart0_tx_busy = 1
 */
void usart0_tx_dma_send(uint8_t *buf, uint16_t len) {
  dma_parameter_struct dma_para;

  while (usart0_tx_busy)
    ; /* 等上一次发完 */

  rcu_periph_clock_enable(RCU_DMA);

  dma_deinit(DMA_CH1);
  dma_struct_para_init(&dma_para);

  dma_para.direction = DMA_MEMORY_TO_PERIPHERAL;
  dma_para.memory_addr = (uint32_t)buf;
  dma_para.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
  dma_para.memory_width = DMA_MEMORY_WIDTH_8BIT;
  dma_para.number = len;
  dma_para.periph_addr = (uint32_t)&USART_TDATA(USART0);
  dma_para.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
  dma_para.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
  dma_para.priority = DMA_PRIORITY_HIGH;
  dma_init(DMA_CH1, &dma_para);

  dma_circulation_disable(DMA_CH1);
  dma_memory_to_memory_disable(DMA_CH1);

  usart0_tx_busy = 1;

  /* 使能 DMA 传输完成中断，用于清除 busy 标志 */
  dma_interrupt_enable(DMA_CH1, DMA_INT_FTF);
  nvic_irq_enable(DMA_Channel1_2_IRQn, 0, 1);

  usart_dma_transmit_config(USART0, USART_TRANSMIT_DMA_ENABLE);
  dma_channel_enable(DMA_CH1);
}

/**
 * @brief DMA 发送完成中断（DMA_CH1 与 DMA_CH2 共用）
 * 清除 usart0_tx_busy，允许下一帧发送
 */
void DMA_Channel1_2_IRQHandler(void) {
  if (dma_interrupt_flag_get(DMA_CH1, DMA_INT_FLAG_FTF) == SET) {
    dma_interrupt_flag_clear(DMA_CH1, DMA_INT_FLAG_FTF);
    dma_channel_disable(DMA_CH1);
    usart_dma_transmit_config(USART0, USART_TRANSMIT_DMA_DISABLE);
    usart0_tx_busy = 0;
  }

  if (dma_interrupt_flag_get(DMA_CH2, DMA_INT_FLAG_FTF) == SET) {
    dma_interrupt_flag_clear(DMA_CH2, DMA_INT_FLAG_FTF);
  }
}

void timer_config(void) {
  timer_parameter_struct timer_initpara;

  /* 1. 使能 TIMER1 时钟 */
  rcu_periph_clock_enable(RCU_TIMER1);

  /* 2. 复位 TIMER1 */
  timer_deinit(TIMER1);

  /* 3. 配置 TIMER1 时基单元 */
  timer_struct_para_init(&timer_initpara);
  timer_initpara.prescaler =
      8400 - 1; // 预分频值 (系统时钟84MHz，分频后为10kHz)
  timer_initpara.alignedmode = TIMER_COUNTER_EDGE;    // 边缘对齐模式
  timer_initpara.counterdirection = TIMER_COUNTER_UP; // 向上计数模式
  timer_initpara.period = 100 - 1; // 自动重装载值 (100个计数周期 = 10ms)
  timer_initpara.clockdivision = TIMER_CKDIV_DIV1; // 时钟分频
  timer_initpara.repetitioncounter = 0;            // 重复计数器设为0
  timer_init(TIMER1, &timer_initpara);

  /* 4. 使能 TIMER1 更新中断 */
  timer_interrupt_enable(TIMER1, TIMER_INT_UP);

  /* 5. 配置 NVIC 中断优先级并使能中断 */
  nvic_irq_enable(TIMER1_IRQn, 0, 1);

  /* 6. 启动定时器 */
  timer_enable(TIMER1);
}

int main(void) {
  systick_config();
  com_usart_init();
  // printf("Hellow word!\n");
  // sprintf(transmitter_buffer, "HELLO_world!\n");
  // usart_interrupt_enable(USART0, USART_INT_TBE);
  // delay_1ms(10);

  // txcount = 0;
  // usart_interrupt_enable(USART0, USART_INT_TBE);
  // printf("hello_word");
  /* configure RCU */
  /* I2C 软模拟初始化（soft_i2c_init 内部已配置 GPIO 和时钟） */
  soft_i2c_init();
  // printf("I2C init done!\n");

  // i2c_test();

  // // soft_i2c_test();

  // gpio_config();
  // i2c_config();

  // i2c_test();

  imu_system_init();
  usart0_rx_dma_idle_init(); /* 配置USART0 DMA接收 + 空闲中断 */
  timer_config();
  // printf("timer init done!\n");

  static uint32_t debug_cnt = 0;
  while (1) {
    if (imu_loop_flag) {
      imu_main_loop(12);
      imu_loop_flag = 0;

      debug_cnt++;
      if (debug_cnt % 10 == 0) {
        // proto_send(usart0_rx_buffer[2]);
        // printf("imu_tmp = %.4f\r\n", icm_raw.temp);
        // printf("mag_norm=%.4f,mag_x=%.4f,mag_y=%.4f,mag_z=%.4f\n",
        //        mag_raw.mag_norm, mag_raw.mx, mag_raw.my, mag_raw.mz);
        printf("P=%.4f,R=%.4f,Y=%.4f\r\n", ahrs.att.pitch, ahrs.att.roll, ahrs.att.yaw_now);
        // printf("ax=%.4f,ay=%.4f,az=%.4f\r\ngx=%.4f,gy=%.4f,gz=%.4f\r\n",
        //        icm_raw.ax, icm_raw.ay, icm_raw.az, icm_raw.gx, icm_raw.gy,
        //        icm_raw.gz);
        // printf("gx=%.4f,gy=%.4f,gz=%.4f\r\n", icm_raw.gx, icm_raw.gy,
        //        icm_raw.gz);
      }
    }
    // 处理串口数据（空闲中断已计算 usart0_rx_len）
    if (usart0_rx_flag) {
      usart0_rx_flag = 0;
      // if (usart0_rx_len >= 3) {
      //   uint8_t calc_cs =
      //       calc_checksum(usart0_rx_buffer + 1, usart0_rx_len - 2);
      //   if (calc_cs != usart0_rx_buffer[usart0_rx_len - 1]) {
      //     /* 校验失败，丢弃此帧 */
      //   } else {
      //     // proto_send(usart0_rx_buffer[2]);
      //   }
      // }
    }
  }
}

// #include "gd32f3x0.h"
// #include "stdio.h"
// #include "systick.h"

// #define delay_ms(x) delay_1ms(x)

// void com_usart_init(void)
// {
//     rcu_periph_clock_enable(RCU_GPIOA);
//     rcu_periph_clock_enable(RCU_USART0);

//     gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_9);
//     gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_10);

//     gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
//     gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ,
//     GPIO_PIN_9);

//     gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
//     gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ,
//     GPIO_PIN_10);

//     usart_deinit(USART0);
//     usart_word_length_set(USART0, USART_WL_8BIT);
//     usart_stop_bit_set(USART0, USART_STB_1BIT);
//     usart_parity_config(USART0, USART_PM_NONE);
//     usart_baudrate_set(USART0, 115200U);
//     usart_receive_config(USART0, USART_RECEIVE_ENABLE);
//     usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
//     usart_enable(USART0);
// }

// void timer_config(void)
// {
//     timer_parameter_struct timer_initpara;

//     rcu_periph_clock_enable(RCU_TIMER1);
//     timer_deinit(TIMER1);

//     timer_struct_para_init(&timer_initpara);
//     timer_initpara.prescaler = 8400 - 1;
//     timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
//     timer_initpara.counterdirection = TIMER_COUNTER_UP;
//     timer_initpara.period = 10000 - 1;
//     timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
//     timer_initpara.repetitioncounter = 0;
//     timer_init(TIMER1, &timer_initpara);

//     timer_interrupt_enable(TIMER1, TIMER_INT_UP);
//     nvic_irq_enable(TIMER1_IRQn, 0, 1);
//     timer_enable(TIMER1);
// }

// volatile uint32_t timer_count = 0;

// void TIMER1_IRQHandler(void)
// {
//     if (timer_interrupt_flag_get(TIMER1, TIMER_INT_FLAG_UP) == SET)
//     {
//         timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_UP);
//         timer_count++;
//     }
// }

// void main(void)
// {
//     systick_config();

//     printf("Step 1: Initializing USART...\n");
//     com_usart_init();
//     printf("Step 2: USART init done!\n");

//     printf("Step 3: Initializing TIMER1...\n");
//     timer_config();
//     printf("Step 4: TIMER1 init done!\n");

//     printf("Step 5: Entering main loop...\n");
//     while (1)
//     {
//         if (timer_count >= 100)
//         {
//             timer_count = 0;
//             printf("Timer interrupt working! Count: %lu\n", timer_count);
//         }
//     }
// }
