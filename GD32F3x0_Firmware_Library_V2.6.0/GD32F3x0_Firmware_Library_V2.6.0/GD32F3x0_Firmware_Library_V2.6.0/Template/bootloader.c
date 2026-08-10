/*!
    \file    bootloader.c
    \brief   OTA Bootloader for GD32F330
    \version 2026-07-31, V1.0.0

    Flash 布局：
      0x08000000 - 0x08000FFF  Bootloader (4KB)
      0x08001000 - 0x08014FFF  App 运行区 (~84KB)
      0x08015000 - 0x0801FFFF  下载区/OTA (44KB)
      0x0800EC00 - 0x0800FFFF  数据分区 (YAW/报警/零点)，OTA 不碰
*/

#include "gd32f3x0.h"
#include "gd32f3x0_rcu.h"
#include "gd32f3x0_gpio.h"
#include "gd32f3x0_usart.h"
#include "gd32f3x0_fmc.h"
#include "bootloader.h"
#include "ota_protocol.h"
#include <stdio.h>

/* ========== 重定向 printf 到 USART0 ========== */
int fputc(int ch, FILE *f) {
  while (RESET == usart_flag_get(USART0, USART_FLAG_TBE))
    ;
  usart_data_transmit(USART0, (uint8_t)ch);
  return ch;
}

/* ========== 系统时钟初始化（与 App 保持一致） ========== */
static void bootloader_system_clock_config(void) {
  /* GD32F330 默认内部 8MHz IRC8M，倍频到 72MHz */
  rcu_osci_on(RCU_IRC8M);
  rcu_osci_stab_wait(RCU_IRC8M);
  rcu_system_clock_source_config(RCU_CKSYSSRC_IRC8M);

  rcu_pll_config(RCU_PLLSRC_IRC8M_DIV2, RCU_PLL_MUL18); /* 8M/2*18 = 72MHz */
  rcu_osci_on(RCU_PLL_CK);
  rcu_osci_stab_wait(RCU_PLL_CK);

  rcu_ahb_clock_config(RCU_AHB_CKSYS_DIV1);
  rcu_apb2_clock_config(RCU_APB2_CKAHB_DIV1);
  rcu_apb1_clock_config(RCU_APB1_CKAHB_DIV2);

  rcu_system_clock_source_config(RCU_CKSYSSRC_PLL);
}

/* ========== USART0 初始化（115200, 8N1） ========== */
static void bootloader_usart_init(void) {
  rcu_periph_clock_enable(RCU_GPIOA);
  rcu_periph_clock_enable(RCU_USART0);

  gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_9);  /* PA9  = USART0_TX */
  gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_10); /* PA10 = USART0_RX */

  gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
  gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
  gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);

  usart_deinit(USART0);
  usart_baudrate_set(USART0, 115200U);
  usart_word_length_set(USART0, USART_WL_8BIT);
  usart_stop_bit_set(USART0, USART_STB_1BIT);
  usart_parity_config(USART0, USART_PM_NONE);
  usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);
  usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);
  usart_receive_config(USART0, USART_RECEIVE_ENABLE);
  usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
  usart_enable(USART0);
}

/* ========== 检查 App 区是否有效 ========== */
uint8_t bootloader_is_app_valid(void) {
  uint32_t app_sp = *((volatile uint32_t *)APP_START_ADDR);
  uint32_t app_pc = *((volatile uint32_t *)(APP_START_ADDR + 4));

  /* 栈顶必须在 SRAM 范围内 (0x20000000 - 0x20003FFF for 16KB SRAM) */
  if (app_sp < 0x20000000U || app_sp > 0x20004000U)
    return 0;

  /* PC 必须在 Flash 范围内且是奇数（Thumb 指令） */
  if ((app_pc & 0x08000000U) == 0 || (app_pc & 1U) == 0)
    return 0;

  return 1;
}

/* ========== 跳转到 App 程序 ========== */
void bootloader_jump_to_app(void) {
  uint32_t app_sp = *((volatile uint32_t *)APP_START_ADDR);
  uint32_t app_pc = *((volatile uint32_t *)(APP_START_ADDR + 4));

  /* 禁用所有中断 */
  __disable_irq();

  /* 重定位向量表 */
  SCB->VTOR = APP_START_ADDR;

  /* 设置栈指针和 PC */
  __set_MSP(app_sp);
  __set_PSP(app_sp);

  /* 跳转到 App 复位处理函数（清除 Thumb 位） */
  void (*app_reset_handler)(void) = (void (*)(void))(app_pc & ~1U);
  app_reset_handler();
}

/* ========== Bootloader 主循环 ========== */
void bootloader_main_loop(void) {
  /* 检查是否有待安装的升级固件 */
  if (ota_check_pending_upgrade()) {
    printf("[BL] pending upgrade detected, installing...\r\n");
    ota_install_firmware();
  }

  /* 检查 App 是否有效 */
  if (bootloader_is_app_valid()) {
    printf("[BL] jumping to app at 0x%08lX\r\n", APP_START_ADDR);
    bootloader_jump_to_app();
  }

  /* App 无效，进入 OTA 等待模式 */
  printf("[BL] no valid app, waiting for OTA...\r\n");

  /* 初始化外设 */
  bootloader_system_clock_config();
  bootloader_usart_init();

  /* 主循环：等待上位机下发 OTA 命令 */
  while (1) {
    if (RESET != usart_flag_get(USART0, USART_FLAG_RBNE)) {
      uint8_t byte = (uint8_t)usart_data_receive(USART0);
      ota_protocol_parse(byte);
    }
  }
}