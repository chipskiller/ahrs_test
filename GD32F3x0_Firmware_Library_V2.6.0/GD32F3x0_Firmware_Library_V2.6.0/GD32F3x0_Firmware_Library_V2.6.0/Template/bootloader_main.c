/*!
    \file    bootloader_main.c
    \brief   OTA Bootloader 入口程序

    【什么是 Bootloader？】
    Bootloader 是一段在 App（主程序）运行之前执行的小程序，类似于电脑的 BIOS。
    它的主要职责：
    1. 检查是否有新固件需要安装（OTA 升级）
    2. 如果有，将新固件从下载区复制到 App 区
    3. 如果没有，直接跳转到 App 运行

    【为什么需要 Bootloader？】
    传统烧录方式：用 JLink/STLink 连接电脑，手动烧录固件到 Flash。
    OTA 烧录方式：设备通过串口/蓝牙/WiFi 接收新固件，自动完成升级。
    
    但是，正在运行的程序不能覆盖自己（就像 Windows 不能删除正在运行的 exe）。
    所以需要 Bootloader 这个"第三方"来完成固件替换工作。

    【工作流程】
    ┌─────────────────────────────────────────────────────────────┐
    │ 1. 设备上电，Bootloader 首先运行                           │
    │ 2. 检查 Flash 中的升级标志                                  │
    │ 3. 如果有升级标志：                                         │
    │    - 将下载区的新固件复制到 App 区                          │
    │    - 清除升级标志                                           │
    │ 4. 检查 App 区是否有效（栈顶和复位向量是否合法）            │
    │ 5. 如果有效：跳转到 App 运行                                │
    │ 6. 如果无效：进入 OTA 等待模式，等待上位机下发新固件        │
    └─────────────────────────────────────────────────────────────┘

    【编译配置】
    - IROM（Flash）: 0x08000000, size 0x1000 (4KB)
    - IRAM（SRAM） : 0x20000000, size 0x2000 (8KB)
    - 定义宏: GD32F330, USE_STDPERIPH_DRIVER

    【烧录方式】
    第一次烧录（必须用编程器）：
      1. 用 JLink/STLink 将 bootloader.bin 烧到 0x08000000
      2. 用 JLink/STLink 将 app.bin 烧到 0x08002000（App 起始地址，见 APP_START_ADDR）
    
    后续升级（通过 OTA）：
      3. 通过串口下发 OTA 升级包，Bootloader 自动完成安装
*/

#include "gd32f3x0.h"
#include "gd32f3x0_rcu.h"
#include "gd32f3x0_gpio.h"
#include "gd32f3x0_usart.h"
#include "gd32f3x0_fmc.h"
#include "ota_protocol.h"
#include <stdio.h>

/* ========== Bootloader 配置 ========== */
#define APP_START_ADDR 0x08002000U /* App 起始地址（第 8 页），固件运行位置 */

/* ========== 重定向 printf 到 USART0 ========== */
/*
 * 功能：让 printf 的输出通过串口发送，而不是屏幕
 * 原理：C 标准库的 printf 会调用 fputc 输出每个字符，我们重写 fputc 即可
 */
int fputc(int ch, FILE *f) {
  while (RESET == usart_flag_get(USART0, USART_FLAG_TBE)) /* 等待发送缓冲区空 */
    ;
  usart_data_transmit(USART0, (uint8_t)ch);
  return ch;
}

/* ========== 系统时钟说明 ========== */
/*
 * 系统时钟由 startup 的 SystemInit()（system_gd32f3x0.c）配置为 84MHz。
 * bootloader 不再重复配置 PLL——否则会与 SystemInit 的 84MHz 冲突，
 * 导致波特率异常，并可能在跳转后让 App 的 SystemInit 重新配时钟时卡死。
 * bootloader 直接使用 SystemInit 的 84MHz，USART0 按 9600 配置。
 */

/* ========== USART0 初始化 ========== */
/*
 * 功能：配置串口 0 为 9600 波特率，8 数据位，1 停止位，无校验
 * 引脚：PA9 = TX（发送），PA10 = RX（接收）
 * 
 * 为什么需要串口？
 * - 打印调试信息（printf）
 * - 接收 OTA 升级数据
 */
static void usart_init(void) {
  /* 使能 GPIOA 和 USART0 时钟 */
  rcu_periph_clock_enable(RCU_GPIOA);
  rcu_periph_clock_enable(RCU_USART0);

  /* 配置 PA9 和 PA10 为复用功能（USART0） */
  gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_9);  /* PA9  = USART0_TX */
  gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_10); /* PA10 = USART0_RX */

  /* 配置 GPIO 模式：复用推挽输出，上拉 */
  gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
  gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
  gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);

  /* 配置 USART0 参数 */
  usart_deinit(USART0);
  usart_baudrate_set(USART0, 9600U);        /* 波特率 9600 */
  usart_word_length_set(USART0, USART_WL_8BIT); /* 8 数据位 */
  usart_stop_bit_set(USART0, USART_STB_1BIT);   /* 1 停止位 */
  usart_parity_config(USART0, USART_PM_NONE);   /* 无校验 */
  usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE); /* 禁用 RTS */
  usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE); /* 禁用 CTS */
  usart_receive_config(USART0, USART_RECEIVE_ENABLE); /* 使能接收 */
  usart_transmit_config(USART0, USART_TRANSMIT_ENABLE); /* 使能发送 */
  usart_enable(USART0);
}

/* ========== 检查 App 区是否有效 ========== */
/*
 * 功能：验证 App 区是否有合法的固件
 * 原理：ARM Cortex-M 的向量表前两个字段是：
 *       - 偏移 0x00：栈顶指针（MSP），必须指向 SRAM 范围
 *       - 偏移 0x04：复位向量（Reset Handler），必须指向 Flash 范围且是奇数（Thumb 指令）
 * 
 * 返回值：1=有效，0=无效
 */
static uint8_t is_app_valid(void) {
  /* 读取 App 区的栈顶指针（向量表第一个字段） */
  uint32_t app_sp = *((volatile uint32_t *)APP_START_ADDR);
  /* 读取 App 区的复位向量（向量表第二个字段） */
  uint32_t app_pc = *((volatile uint32_t *)(APP_START_ADDR + 4));

  /* 栈顶必须在 SRAM 范围内 (0x20000000 - 0x20003FFF for 16KB SRAM) */
  /* 如果栈顶不在 SRAM 范围，说明 Flash 可能是空的或损坏的 */
  if (app_sp < 0x20000000U || app_sp > 0x20004000U)
    return 0;

  /* PC 必须在 Flash 范围内（0x08xxxxxx）且是奇数（Thumb 指令集要求） */
  /* ARM Cortex-M 只支持 Thumb 指令，所以复位向量的最低位必须是 1 */
  if ((app_pc & 0x08000000U) == 0 || (app_pc & 1U) == 0)
    return 0;

  return 1; /* 验证通过 */
}

/* ========== 跳转到 App 程序 ========== */
/*
 * 功能：从 Bootloader 跳转到 App 运行
 * 原理：
 *   1. 禁用中断（防止跳转过程中触发中断）
 *   2. 重定向向量表（SCB->VTOR）到 App 区起始地址
 *   3. 设置栈指针（MSP/PSP）为 App 的栈顶
 *   4. 跳转到 App 的复位处理函数（Reset Handler）
 * 
 * 类比：就像电脑 BIOS 完成自检后，将控制权交给硬盘上的操作系统
 */
static void jump_to_app(void) {
  /* 读取 App 的栈顶指针和复位向量 */
  uint32_t app_sp = *((volatile uint32_t *)APP_START_ADDR);
  uint32_t app_pc = *((volatile uint32_t *)(APP_START_ADDR + 4));

  /* 禁用所有中断（防止跳转过程中触发中断导致崩溃） */
  __disable_irq();

  /* 重定向向量表到 App 区 */
  /* 向量表存放中断处理函数地址，App 和 Bootloader 的向量表不同 */
  SCB->VTOR = APP_START_ADDR;

  /* 设置主栈指针（MSP）和进程栈指针（PSP） */
  /* MSP 用于异常处理，PSP 用于线程模式（RTOS 中使用） */
  __set_MSP(app_sp);
  __set_PSP(app_sp);

  /* 跳转到 App 复位处理函数 */
  /* 关键：必须保留 Thumb 位（app_pc 最低位为 1）。
     Cortex-M 只支持 Thumb 指令，BLX/BX 会用地址最低位切换状态：
     bit0=1 → Thumb（正确）；bit0=0 → ARM 状态 → UsageFault/HardFault！ */
  void (*app_reset_handler)(void) = (void (*)(void))app_pc;
  app_reset_handler(); /* 执行跳转，从此处开始运行 App */
}

/* ========== 主函数 ========== */
/*
 * Bootloader 的主流程：
 * 1. 检查是否有待安装的升级固件
 * 2. 如果有，执行安装（复制固件到 App 区）
 * 3. 检查 App 是否有效
 * 4. 如果有效，跳转到 App
 * 5. 如果无效，进入 OTA 等待模式
 */
int main(void) {
  /* 步骤 0：先初始化串口——printf 依赖 USART0。
     系统时钟已由 startup 的 SystemInit 配置为 84MHz，bootloader 不再重复配置 PLL */
  usart_init();
  printf("[BL] Bootloader started\r\n");

  /* 步骤 1：检查是否有待安装的升级固件 */
  if (ota_check_pending_upgrade()) {
    printf("[BL] pending upgrade detected, installing...\r\n");
    ota_install_firmware();
  }

  /* 步骤 2：检查 App 区是否有效 */
  if (is_app_valid()) {
    uint32_t app_sp = *((volatile uint32_t *)APP_START_ADDR);
    uint32_t app_pc = *((volatile uint32_t *)(APP_START_ADDR + 4));
    printf("[BL] jumping: sp=0x%08lX reset=0x%08lX\r\n", app_sp, app_pc);
    jump_to_app();
  }

  /* 步骤 3：App 无效，进入 OTA 等待模式 */
  printf("[BL] no valid app, waiting for OTA...\r\n");

  /* 主循环：等待上位机下发 OTA 命令 */
  while (1) {
    if (RESET != usart_flag_get(USART0, USART_FLAG_RBNE)) {
      uint8_t byte = (uint8_t)usart_data_receive(USART0);
      ota_protocol_parse(byte);
    }
  }
}