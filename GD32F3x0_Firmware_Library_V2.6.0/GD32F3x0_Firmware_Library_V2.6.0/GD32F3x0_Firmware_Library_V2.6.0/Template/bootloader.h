#ifndef BOOTLOADER_H
#define BOOTLOADER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== Bootloader 配置 ========== */
#define BOOTLOADER_SIZE    0x00001000U  /* 4KB */
#define APP_START_ADDR     0x08001000U  /* App 起始地址（第 4 页） */

/* ========== API ========== */

/* 检查 App 区是否有效（栈顶指针在 SRAM 范围内） */
uint8_t bootloader_is_app_valid(void);

/* 跳转到 App 程序 */
void bootloader_jump_to_app(void);

/* Bootloader 主循环（接收 OTA 命令） */
void bootloader_main_loop(void);

#ifdef __cplusplus
}
#endif

#endif /* BOOTLOADER_H */