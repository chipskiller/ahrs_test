/*!
    \file    ota_protocol.h
    \brief   OTA（Over-The-Air）远程固件升级协议 — 类型定义与常量

    【通信协议帧格式】
    请求帧：[帧头 0xAA] [帧长] [功能码] [数据...] [校验码]
    回复帧：[帧头 0xAA] [帧长] [功能码] [状态] [数据...] [校验码]

    【功能码说明】
    0xF0  开始升级：上位机 → 设备，携带版本号 + 总包数，设备擦除下载区
    0xF1  写入数据：上位机 → 设备，每包 512 字节，设备写入 Flash 并回读校验
    0xF2  结束升级：上位机 → 设备，携带总校验码，设备写入升级标志后重启
    0xF3  查询版本：上位机 → 设备，设备回复当前固件版本号

    【Flash 分区布局】
    0x08000000 - 0x08000FFF  Bootloader (4KB)
    0x08001000 - 0x08007FFF  App 运行区 (32KB)
    0x08008000 - 0x0800DFFF  下载区 (24KB)
    0x0800E000 - 0x0801FFFF  数据分区 (~80KB) — OTA 升级不触碰
*/

#ifndef OTA_PROTOCOL_H
#define OTA_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========== 帧格式常量 ========== */

#define OTA_HEAD            0xAAU   /* 帧头 */
#define OTA_RSP_HEAD        0xAAU   /* 回复帧头（与请求帧头相同） */

/* ========== 功能码 ========== */

#define OTA_CMD_START       0xF0U   /* 开始升级：擦除下载区，准备接收 */
#define OTA_CMD_WRITE       0xF1U   /* 写入数据：分包写入 Flash */
#define OTA_CMD_FINISH      0xF2U   /* 结束升级：校验并写入升级标志 */
#define OTA_CMD_VERSION     0xF3U   /* 查询版本：回复当前固件版本号 */

/* ========== 状态码 ========== */

#define OTA_OK              0x00U   /* 操作成功 */
#define OTA_FAIL             0xFFU   /* 操作失败 */

/* ========== 数据包配置 ========== */

#define OTA_PKT_DATA_SIZE   512U    /* 每包数据大小（字节） */
#define OTA_VERSION_LEN      16U    /* 版本号字符串长度（含结尾 '\0'） */

/* ========== 升级标志 ========== */

/*
 * 升级标志存储在数据分区的第 60 页（0x0801F000），
 * 与 YAW/报警/零点等数据分区不冲突。
 * Bootloader 启动时读取此地址的魔数，判断是否有待安装的固件。
 */
#define OTA_FLAG_ADDR       0x0801F000U  /* 升级标志存储地址 */
#define OTA_FLAG_MAGIC      0xAA55AA55U  /* 魔数：标识有效的升级标志 */

/* ========== 类型定义 ========== */

/*
 * 升级标志结构体（16 字节，存储在 Flash 中）
 * 上位机下发 0xF2（结束升级）命令后，设备将此结构体写入 OTA_FLAG_ADDR，
 * 然后重启。Bootloader 检测到有效魔数后执行固件安装。
 */
typedef struct {
    uint32_t magic;           /* 魔数，必须等于 OTA_FLAG_MAGIC */
    uint32_t total_checksum;  /* 总校验码（所有数据包的累加校验和） */
    uint32_t total_packets;   /* 固件总包数 */
    uint32_t reserved;        /* 保留字段（暂未使用） */
} ota_flag_t;

/*
 * OTA 状态枚举
 * IDLE：     空闲，等待升级命令
 * RECEIVING：正在接收固件数据包
 */
typedef enum {
    OTA_STATE_IDLE = 0,       /* 空闲状态 */
    OTA_STATE_RECEIVING       /* 接收状态 */
} ota_state_t;

/* ========== 全局变量（外部声明） ========== */

extern volatile ota_state_t  ota_state;          /* OTA 当前状态 */
extern volatile uint32_t     ota_packet_count;   /* 已接收包数 */
extern volatile uint32_t     ota_total_packets;  /* 固件总包数 */
extern volatile uint32_t     ota_total_checksum; /* 累计校验码 */
extern volatile uint8_t      ota_rx_buffer[OTA_PKT_DATA_SIZE]; /* 接收缓冲区 */
extern volatile uint16_t     ota_rx_len;         /* 已接收长度 */

/* ========== API 函数声明 ========== */

/* 发送回复帧 */
void ota_send_response(uint8_t cmd, uint8_t status,
                       const uint8_t *data, uint8_t data_len);

/* 获取固件版本号 */
const char *ota_get_version(void);

/* 协议解析入口（逐字节调用） */
void ota_protocol_parse(uint8_t byte);

/* 检查是否有待安装的升级固件 */
uint8_t ota_check_pending_upgrade(void);

/* 执行固件安装（下载区 → App 区） */
void ota_install_firmware(void);

#ifdef __cplusplus
}
#endif

#endif /* OTA_PROTOCOL_H */