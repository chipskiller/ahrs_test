/*!
    \file    ota_protocol.c
    \brief   OTA（Over-The-Air）远程固件升级协议实现

    【什么是 OTA？】
    OTA 全称 "Over-The-Air"，即空中下载技术。通过串口/蓝牙/WiFi 等通信方式，
    远程更新设备的固件，而不需要用编程器（JLink/STLink）重新烧录。

    【OTA 工作流程】
    1. 上位机（电脑/手机）发送 "开始升级" 命令，包含固件版本号和总包数
    2. 设备擦除 Flash 下载区，回复 "准备就绪"
    3. 上位机分包发送固件数据（每包 512 字节），设备逐包写入 Flash 并校验
    4. 上位机发送 "升级结束" 命令，包含总校验码
    5. 设备校验通过后，写入升级标志到 Flash，然后重启
    6. 重启后 Bootloader 检测到升级标志，将下载区固件复制到 App 区，完成升级

    【Flash 分区布局】
    ┌─────────────────────────────────────────────────────────────┐
    │ 0x08000000 - 0x08001FFF  Bootloader (8KB)                   │
    │ 0x08002000 - 0x08008FFF  App 运行区 (28KB) ← 你的固件       │
    │ 0x08009000 - 0x0800FFFF  下载区 (28KB) ← OTA 临时存放新固件 │
    │ 0x08010000 - 0x0801FFFF  数据分区 (64KB) ← 保存 yaw 等数据│
    └─────────────────────────────────────────────────────────────┘
    ★ 重要：OTA 升级时只擦除 App 区，数据分区绝对安全！

    【通信协议帧格式】
    帧头(1B) + 帧长(1B) + 功能码(1B) + 数据(NB) + 校验码(1B)
    - 帧头：固定 0xAA
    - 帧长：功能码 + 数据的总字节数
    - 功能码：0xF0(开始) / 0xF1(写入) / 0xF2(结束) / 0xF3(查询版本)
    - 校验码：从帧长到数据末尾，逐字节异或后取反

    【状态机】
    OTA_STATE_IDLE       → 空闲状态，等待升级命令
    OTA_STATE_RECEIVING  → 正在接收固件数据
*/

#include "ota_protocol.h"
#include "gd32f3x0_fmc.h"
#include "gd32f3x0_usart.h"
#include "systick.h"
#include <stdio.h>
#include <string.h>

/* ========== 固件版本号 ========== */
/* 上位机通过对比版本号判断是否需要升级，版本相同则拒绝升级 */
#define FW_VERSION "AHRS_V2.0.2_beta"

/* ========== Flash 地址定义 ========== */
/* 这些地址必须与 flash.h 中的分区定义保持一致 */
#define APP_START_ADDR                                                         \
  0x08002000U /* App 区起始地址（第 8 页），当前运行的固件在这里 */
#define APP_END_ADDR                                                           \
  0x08008FFFU /* App 区结束地址（第 35 页），升级时会被擦除 */
#define DOWNLOAD_START_ADDR                                                    \
  0x08009000U /* 下载区起始地址（第 36 页），临时存放新固件 */
#define DOWNLOAD_SIZE 0x00007000U /* 下载区大小 28KB（第 36-63 页） */
#define DATA_ZONE_START                                                        \
  0x08010000U /* 数据分区起始（第 64 页），OTA 绝不触碰              \
               */

/* ========== 全局变量 ========== */
/* volatile 关键字告诉编译器这些变量可能被中断修改，不要优化掉 */
volatile ota_state_t ota_state = OTA_STATE_IDLE;   /* OTA 当前状态 */
volatile uint32_t ota_packet_count = 0;            /* 已接收的包数 */
volatile uint32_t ota_total_packets = 0;           /* 固件总包数 */
volatile uint8_t ota_total_checksum = 0;           /* 累计校验码（1 字节） */
volatile uint8_t ota_rx_buffer[OTA_PKT_DATA_SIZE]; /* 接收缓冲区 */
volatile uint16_t ota_rx_len = 0;                  /* 已接收长度 */

/* ========== 帧解析缓冲区 ========== */
/* 用于暂存从串口收到的数据帧，最大 600 字节 */
static uint8_t parse_buf[600];
volatile static uint16_t parse_idx = 0; /* 当前写入位置 */
static uint8_t parse_state = 0; /* 解析状态：0=等待帧头, 1=收帧长, 2=收数据 */

/* ========== 校验码计算 ========== */
/*
 * 算法：从帧长字节开始，逐字节异或（XOR），最后取反
 * 例如：数据为 [0x03, 0xF0, 0x01]
 *       异或：0x03 ^ 0xF0 ^ 0x01 = 0xF2
 *       取反：~0xF2 = 0x0D
 *       校验码 = 0x0D
 *
 * 作用：检测传输过程中是否有数据损坏（比特翻转、丢包等）
 */
static uint8_t ota_calc_checksum(const uint8_t *data, uint16_t len) {
  uint8_t xor_val = 0;
  for (uint16_t i = 0; i < len; i++) {
    xor_val ^= data[i];
  }
  return ~xor_val;
}

/* ========== 发送回复帧 ========== */
/*
 * 功能：向串口发送回复帧，告知上位机操作结果
 * 参数：
 *   cmd      - 功能码（0xF0/F1/F2/F3）
 *   status   - 状态码（OTA_OK=成功, OTA_FAIL=失败）
 *   data     - 回复数据指针（可为 NULL）
 *   data_len - 数据长度
 *
 * 帧格式：[帧头 0xAA] [帧长] [功能码] [状态] [数据...] [校验码]
 */
void ota_send_response(uint8_t cmd, uint8_t status, const uint8_t *data,
                       uint8_t data_len) {
  uint8_t tx_buf[600];
  uint16_t idx = 0;

  tx_buf[idx++] = OTA_RSP_HEAD; /* 帧头 */
  tx_buf[idx++] = 0;            /* 帧长 = 功能码 + 状态 + 数据 */
  tx_buf[idx++] = cmd;          /* 功能码 */
  if (data != NULL && data_len > 0) {
    memcpy(&tx_buf[idx], data, data_len); /* 复制数据 */
    idx += data_len;
  }
  tx_buf[idx++] = status; /* 状态码 */
  tx_buf[1] =
      idx -
      1; /* 帧长 = 总帧长-2 = 功能码+数据+状态+校验码（不含帧头和帧长本身） */
  uint8_t cs =
      ota_calc_checksum(&tx_buf[1], idx - 1); /* 计算校验码（从帧长到状态） */
  tx_buf[idx++] = cs;

  /* 阻塞发送（升级过程中允许阻塞，因为此时不需要运行其他功能） */
  for (uint16_t i = 0; i < idx; i++) {
    while (RESET ==
           usart_flag_get(USART0, USART_FLAG_TBE)) /* 等待发送缓冲区空 */
      ;
    usart_data_transmit(USART0, tx_buf[i]);
  }
}

/* ========== 获取版本号 ========== */
const char *ota_get_version(void) { return FW_VERSION; }

/* ========== 0xF0 开始升级 ========== */
/*
 * 上位机发送此命令启动升级流程
 * 帧格式: [AA] [14] [F0] [总包数高] [总包数低] [版本号16字节] [校验码]
 *
 * 处理流程：
 * 1. 检查版本号，如果与当前固件相同则拒绝升级（防止重复烧录）
 * 2. 记录总包数，重置计数器
 * 3. 擦除下载区 Flash，准备接收新固件
 * 4. 回复上位机 "准备就绪"
 */
void ota_handle_start(uint8_t *frame, uint8_t len) {
  /* 帧长度至少 22 字节：1(帧头) + 1(帧长) + 1(功能码) + 2(总包数) + 16(版本号)
   * + 1(校验码) */
  if (len < 22) {
    ota_send_response(OTA_CMD_START, OTA_FAIL, NULL, 0);
    return;
  }

  /* 提取版本号（跳过帧头+帧长+功能码+总包数2字节 = 偏移5） */
  char recv_version[17] = {0};
  memcpy(recv_version, &frame[5], 16);

  /* 对比版本号 */
  if (strcmp(recv_version, FW_VERSION) == 0) {
    /* 版本相同，禁止升级 */
    // uint8_t rsp_data[1] = { OTA_FAIL };
    ota_send_response(OTA_CMD_START, OTA_FAIL, NULL, 0);
    // printf("[OTA] version same, reject upgrade\r\n");
    return;
  }

  /* 版本不同，允许升级 */
  ota_total_packets =
      ((uint32_t)frame[3] << 8) | frame[4]; /* 组合高8位和低8位 */
  ota_packet_count = 0;
  ota_total_checksum = 0;
  ota_state = OTA_STATE_RECEIVING; /* 切换到接收状态 */

  /* 擦除下载区（第 12-19 页，每页 1024 字节） */
  /* 擦除 Flash 是写入的前提，Flash 只能从 1 写为 0，不能从 0 写为 1 */
  fmc_unlock(); /* 解锁 Flash 控制器 */
  for (uint32_t page = DOWNLOAD_START_ADDR;
       page < DOWNLOAD_START_ADDR + DOWNLOAD_SIZE; page += 1024) {
    fmc_page_erase(page); /* 逐页擦除 */
  }
  fmc_lock(); /* 重新锁定，防止误操作 */

  // uint8_t rsp_data[1] = {OTA_OK};
  ota_send_response(OTA_CMD_START, OTA_OK, NULL, 0);
  // printf("[OTA] start upgrade, total packets=%lu\r\n", ota_total_packets);
}

/* ========== 0xF1 升级写入 ========== */
/*
 * 上位机分包发送固件数据，每包 512 字节
 * 帧格式: [AA] [00] [F1] [包号高] [包号低] [512字节数据] [校验码]
 *
 * 处理流程：
 * 1. 检查包号是否连续（防止丢包或乱序）
 * 2. 将数据写入 Flash 下载区
 * 3. 回读校验（写入后立即读取，确认写入成功）
 * 4. 回复上位机 "写入成功" 或 "写入失败"
 */
void ota_handle_write(uint8_t *frame, uint16_t len) {
  // printf("[DBG] ota_write: len=%u f=%02X %02X %02X %02X %02X %02X\r\n", len,
  //        frame[0], frame[1], frame[2], frame[3], frame[4], frame[5]);
  /* 必须处于接收状态才允许写入 */
  if (ota_state != OTA_STATE_RECEIVING) {
    ota_send_response(OTA_CMD_WRITE, OTA_FAIL, NULL, 0);
    // printf("[OTA] not in receiving state\r\n");
    return;
  }

  /* 帧长度至少 518 字节：1(帧头) + 1(帧长) + 1(功能码) + 2(包号) + 512(数据) +
   * 1(校验码) */
  if (len < 518) {
    ota_send_response(OTA_CMD_WRITE, OTA_FAIL, NULL, 0);
    // printf("[OTA] frame length too short\r\n");
    return;
  }

  /* 提取包号（高8位和低8位组合） */
  uint16_t pkt_num = ((uint16_t)frame[3] << 8) | frame[4];

  /* 校验包号连续性：期望的包号必须等于实际收到的包号 */
  if (pkt_num != ota_packet_count) {
    ota_send_response(OTA_CMD_WRITE, OTA_FAIL, NULL, 0);
    // printf("[OTA] packet mismatch: expect=%lu, got=%u\r\n", ota_packet_count,
    //       pkt_num);
    return;
  }

  /* 计算写入地址：下载区起始 + 包号 × 每包大小 */
  uint32_t write_addr = DOWNLOAD_START_ADDR + pkt_num * OTA_PKT_DATA_SIZE;

  /* 安全检查：防止越界写入（避免写坏其他分区） */
  if (write_addr + OTA_PKT_DATA_SIZE > DOWNLOAD_START_ADDR + DOWNLOAD_SIZE) {
    ota_send_response(OTA_CMD_WRITE, OTA_FAIL, NULL, 0);
    // printf("[OTA] write out of range\r\n");
    return;
  }

  fmc_unlock(); /* 解锁 Flash */

  /* 逐字写入（每次 4 字节，GD32 Flash 最小写入单位） */
  uint8_t write_ok = 1;
  for (uint16_t i = 0; i < OTA_PKT_DATA_SIZE; i += 4) {
    uint32_t word;
    memcpy(&word, &frame[5 + i], 4);        /* 从帧中提取 4 字节 */
    fmc_word_program(write_addr + i, word); /* 写入 Flash */

    /* 回读校验：写入后立即读取，确认数据正确 */
    uint32_t verify = *((volatile uint32_t *)(write_addr + i));
    if (verify != word) {
      write_ok = 0; /* 校验失败 */
      break;
      // printf("[OTA] write fail at byte %u, expect=0x%08X, got=0x%08X\r\n",
      //        i, word, verify);
    }
  }

  fmc_lock(); /* 锁定 Flash */

  if (write_ok) {
    ota_packet_count++; /* 包数 +1 */
    /* 累计整包校验码：逐字节异或（1 字节）*/
    for (uint16_t i = 0; i < OTA_PKT_DATA_SIZE; i++) {
      ota_total_checksum ^= frame[5 + i];
    }
    ota_send_response(OTA_CMD_WRITE, OTA_OK, NULL, 0);
    // printf("[OTA] write success at packet %u\r\n", pkt_num);
  } else {
    ota_send_response(OTA_CMD_WRITE, OTA_FAIL, NULL, 0);
    // printf("[OTA] write fail at packet %u\r\n", pkt_num);
  }
}

/* ========== 0xF2 升级结束 ========== */
/*
 * 上位机发送此命令表示所有数据包已发送完毕
 * 帧格式: [AA] [03] [F2] [总包校验码1字节] [校验码]
 *
 * 处理流程：
 * 1. 校验总包数是否正确
 * 2. 校验总校验码是否正确
 * 3. 校验通过后，写入升级标志到 Flash（告诉 Bootloader 有新固件待安装）
 * 4. 设备需要重启才能完成安装
 */
void ota_handle_finish(uint8_t *frame, uint8_t len) {
  /* 必须处于接收状态 */
  if (ota_state != OTA_STATE_RECEIVING) {
    ota_send_response(OTA_CMD_FINISH, OTA_FAIL, NULL, 0);
    return;
  }

  /* 帧长度至少 5 字节：1(帧头) + 1(帧长) + 1(功能码) + 1(总包校验码) +
   * 1(帧校验码) */
  if (len < 5) {
    ota_send_response(OTA_CMD_FINISH, OTA_FAIL, NULL, 0);
    return;
  }

  /* 提取上位机发来的总包校验码（1 字节） */
  uint8_t host_checksum = frame[3];

  /* 校验包数：实际接收的包数必须等于上位机声明的总包数 */
  if (ota_packet_count != ota_total_packets) {
    ota_send_response(OTA_CMD_FINISH, OTA_FAIL, NULL, 0);
    // printf("[OTA] packet count mismatch: expect=%lu, got=%lu\r\n",
    //       ota_total_packets, ota_packet_count);
    return;
  }

  /* 校验总包校验码：确保整个固件数据完整。总包检验码不需要取反。*/
  if (host_checksum != ota_total_checksum) {
    ota_send_response(OTA_CMD_FINISH, OTA_FAIL, NULL, 0);
    // printf("[OTA] checksum mismatch: host=0x%02X, calc=0x%02X\r\n",
    //       host_checksum, ota_total_checksum);
    return;
  }

  /* 校验通过，写升级标志到 Flash */
  /* Bootloader 启动时会检查这个标志，如果存在则执行固件安装 */
  ota_flag_t flag;
  flag.magic = OTA_FLAG_MAGIC;            /* 魔数，用于识别有效的升级标志 */
  flag.total_checksum = host_checksum;    /* 总校验码 */
  flag.total_packets = ota_total_packets; /* 总包数 */
  flag.reserved = 0;                      /* 保留字段 */

  //printf("[OTA] writing flag: magic=0x%08X packets=%lu cs=0x%02X\r\n",
        //  flag.magic, flag.total_packets, host_checksum);
  //printf("[OTA] flag addr=0x%08X\r\n", OTA_FLAG_ADDR);

  fmc_unlock();

  /* 擦除并写入 */
  fmc_page_erase(OTA_FLAG_ADDR);
  //printf("[OTA] erase done\r\n");

  fmc_word_program(OTA_FLAG_ADDR, flag.magic);
  //printf("[OTA] prog magic done\r\n");

  fmc_word_program(OTA_FLAG_ADDR + 4, flag.total_checksum);
  fmc_word_program(OTA_FLAG_ADDR + 8, flag.total_packets);
  fmc_word_program(OTA_FLAG_ADDR + 12, flag.reserved);

  fmc_lock();

  /* 回读验证 */
  uint32_t verify_magic = *((volatile uint32_t *)OTA_FLAG_ADDR);
  uint32_t verify_packets = *((volatile uint32_t *)(OTA_FLAG_ADDR + 8));
  //printf("[OTA] verify: magic=0x%08lX packets=%lu\r\n", verify_magic,
  //       verify_packets);

  if (verify_magic != OTA_FLAG_MAGIC) {
    //printf("[OTA] *** FLAG WRITE FAILED! ***\r\n");
    ota_send_response(OTA_CMD_FINISH, OTA_FAIL, NULL, 0);
    return;
  }

  ota_state = OTA_STATE_IDLE; /* 回到空闲状态 */

  ota_send_response(OTA_CMD_FINISH, OTA_OK, NULL, 0);
  //printf("[OTA] upgrade finish OK, reboot to install\r\n");

  /* ★ 关键：写标志完成后，软件复位，让 Bootloader 重新启动并安装固件 */
  /* 等发送完成（TC = Transmission Complete）：阻塞发送只等 TBE（数据进移位寄存器），
     最后一位可能还在 TX 引脚上移出；复位前必须等 TC，避免截断回复帧最后一个字节 */
  while (RESET == usart_flag_get(USART0, USART_FLAG_TC));
  NVIC_SystemReset(); /* 软件复位，等效于上电 */
}

/* ========== 0xF3 查询版本号 ========== */
/* 上位机发送此命令查询当前固件版本，用于判断是否需要升级 */
void ota_handle_version(uint8_t *frame, uint8_t len) {
  const char *ver = ota_get_version();
  ota_send_response(OTA_CMD_VERSION, OTA_OK, (const uint8_t *)ver,
                    OTA_VERSION_LEN);
}

/* ========== 协议解析入口（逐字节调用） ========== */
/**
 * @brief  OTA 协议帧解析状态机（逐字节处理）
 * @param  byte  从串口接收到的单个字节
 * @note   该函数需在串口接收中断或主循环中被调用，每收到一个字节调用一次。
 *         内部使用静态变量维护解析状态，支持异步流式解析，无需等待完整帧再处理。
 *
 * 【状态机流程】
 *   状态 0 (OTA_PARSE_IDLE)：等待帧头 0xAA
 *   状态 1 (OTA_PARSE_LEN)：接收帧长字节
 *   状态 2 (OTA_PARSE_DATA)：接收功能码、数据域及校验码
 *
 * 【帧格式】
 *   [帧头 1B] [帧长 1B] [功能码 1B] [数据 N 字节] [校验码 1B]
 *   - 帧头：固定 0xAA
 *   - 帧长：功能码 + 数据域 + 校验码 的总字节数
 *   - 校验范围：从帧长字节开始，共 frame_len 字节，逐字节异或后取反
 *
 * 【错误处理】
 *   - 帧头不匹配：保持在状态 0 继续等待
 *   - 缓冲区溢出：重置状态机，丢弃当前帧
 *   - 校验失败：打印错误日志，丢弃当前帧
 *   - 未知功能码：静默忽略，不执行任何操作
 */
void ota_protocol_parse(uint8_t byte) {
  switch (parse_state) {

  /* ---------- 状态 0：等待帧头 ---------- */
  case 0:
    /* 检测到帧头 0xAA，开始新一帧的接收 */
    if (byte == OTA_HEAD) {
      parse_buf[0] = byte; /* 保存帧头 */
      parse_idx = 1;       /* 索引指向下一写入位置 */
      parse_state = 1;     /* 进入帧长接收状态 */
    }
    /* 非帧头字节直接丢弃，继续等待 */
    break;

  /* ---------- 状态 1：接收帧长 ---------- */
  case 1:
    parse_buf[1] = byte; /* 保存帧长字节 */
    parse_idx = 2;       /* 索引指向数据域起始位置 */
    parse_state = 2;     /* 进入数据接收状态 */
    break;

  /* ---------- 状态 2：接收数据域及校验码 ---------- */
  case 2:
    /* 缓冲区溢出保护：防止接收超长帧导致内存越界 */
    if (parse_idx < sizeof(parse_buf)) {
      parse_buf[parse_idx++] = byte; /* 存入接收缓冲区 */

      /*
       * 判断当前帧是否接收完整。
       * ★ 注意：0xF1 数据帧的“帧长”字段固定为 0（协议规定），
       *   不能用它判断帧完整性；0xF1 是定长帧，总长固定 518 字节。
       *   其余命令帧（0xF0/0xF2/0xF3）帧长字段有效：
       *   总帧长 = 帧头(1B) + 帧长(1B) + frame_len(功能码+数据+校验码) = 2 +
       * frame_len
       */
      uint16_t expect_total; /* 本帧期望总长度 */
      uint16_t cs_len;       /* 校验码计算长度（从帧长到数据末尾） */
      if (parse_idx >= 3 && parse_buf[2] == OTA_CMD_WRITE) {
        /* 0xF1 定长数据帧：AA + 帧长 + F1 + 包号2B + 数据512B + 校验码 = 518B
         */
        expect_total = (uint16_t)(518); /* = 518 */
        cs_len = (uint16_t)(516);       /* = 516：帧长+功能码+包号+数据 */
      } else {
        expect_total = (uint16_t)(2U + parse_buf[1]);
        cs_len = parse_buf[1];
      }

      if (parse_idx >= expect_total) {
        /* ===== 帧接收完成，进入处理阶段 ===== */
        uint8_t func_code = parse_buf[2]; /* 功能码位于帧头、帧长之后 */
        // printf("[DBG] parse: idx=%u expect=%u func=%02X f0=%02X f1=%02X\r\n",
        //        parse_idx, expect_total, func_code, parse_buf[0],
        //        parse_buf[1]);
        uint8_t recv_cs =
            parse_buf[parse_idx - 1]; /* 校验码始终是最后一个字节 */

        /*
         * 计算校验码：
         * - 起始地址：&parse_buf[1]（从帧长字节开始）
         * - 长度：cs_len（从帧长到数据末尾）
         * - 算法：逐字节异或，最后取反
         */
        uint8_t calc_cs = ota_calc_checksum(&parse_buf[1], cs_len);

        if (recv_cs == calc_cs) {
          /* ----- 校验通过，根据功能码分发命令 ----- */
          switch (func_code) {
          case OTA_CMD_START: /* 0xF0：开始升级 */
            ota_handle_start(parse_buf, parse_idx);
            break;
          case OTA_CMD_WRITE: /* 0xF1：写入固件数据 */
            ota_handle_write(parse_buf, parse_idx);
            break;
          case OTA_CMD_FINISH: /* 0xF2：结束升级 */
            ota_handle_finish(parse_buf, parse_idx);
            break;
          case OTA_CMD_VERSION: /* 0xF3：查询版本号 */
            ota_handle_version(parse_buf, parse_idx);
            break;
          default:
            /* 未知功能码，静默丢弃，不回复 */
            // printf("[OTA] unknown command %02X\r\n", func_code);
            break;
          }
        } else {
          /* 校验失败，记录日志以便调试 */
          // printf("[OTA] checksum error\r\n");
        }

        /* 无论处理成功与否，重置状态机准备接收下一帧 */
        parse_state = 0;
        parse_idx = 0;
      }
    } else {
      /* 缓冲区溢出：当前帧超长，强制重置状态机 */
      parse_state = 0;
      parse_idx = 0;
    }
    break;

  /* ---------- 异常状态恢复 ---------- */
  default:
    /* 状态机异常（如内存被篡改），强制复位到初始状态 */
    parse_state = 0;
    parse_idx = 0;
    break;
  }
}

/* ========== 检查是否有待安装的升级固件 ========== */
/*
 * 功能：Bootloader 启动时调用，检查 Flash 中是否有升级标志
 * 返回值：1=有升级标志，需要安装新固件；0=无升级标志，直接跳转 App
 *
 * 原理：读取 OTA_FLAG_ADDR 地址的魔数，如果等于 OTA_FLAG_MAGIC，
 *       说明上位机已完成固件下载，等待重启安装
 */
uint8_t ota_check_pending_upgrade(void) {
  ota_flag_t flag;
  flag.magic = *((volatile uint32_t *)OTA_FLAG_ADDR);
  flag.total_checksum = *((volatile uint32_t *)(OTA_FLAG_ADDR + 4));
  flag.total_packets = *((volatile uint32_t *)(OTA_FLAG_ADDR + 8));

  //printf("[BL] flag@0x%08X: magic=0x%08lX packets=%lu checksum=0x%02lX\r\n",
  //       OTA_FLAG_ADDR, flag.magic, flag.total_packets, flag.total_checksum);

  if (flag.magic == OTA_FLAG_MAGIC) {
    /* ★ 关键：将 Flash 中的参数赋值给全局变量，
       否则 ota_install_firmware() 中 ota_total_packets 为 0，
       导致固件复制 0 字节，升级无效 */
    ota_total_packets = flag.total_packets;
    ota_total_checksum = (uint8_t)(flag.total_checksum & 0xFF);
    //printf("[BL] loaded: total_packets=%lu\r\n", ota_total_packets);
    return 1; /* 有升级标志 */
  }
  return 0; /* 无升级标志 */
}

/* ========== 执行固件复制（下载区 → App 区） ========== */
/*
 * 功能：将下载区的新固件复制到 App 区，完成升级安装
 * 调用时机：Bootloader 检测到升级标志后调用
 *
 * 处理流程：
 * 1. 擦除 App 区（旧固件）
 * 2. 从下载区逐字复制到 App 区
 * 3. 清除升级标志
 * 4. 重启后跳转到新固件
 *
 * ★ 重要：只擦除 App 区（0x08001000-0x08007FFF），数据分区（0x0800E000
 * 起始）绝对安全！
 */
void ota_install_firmware(void) {
  uint32_t copy_size = ota_total_packets * OTA_PKT_DATA_SIZE;
  //printf("[BL] installing: src=0x%08X dst=0x%08X size=%lu\r\n",
//         DOWNLOAD_START_ADDR, APP_START_ADDR, copy_size);

  /* 擦除 App 区（第 4-11 页，每页 1024 字节） */
  fmc_unlock();
  for (uint32_t page = APP_START_ADDR; page <= APP_END_ADDR; page += 1024) {
    fmc_page_erase(page);
  }
  //printf("[BL] app region erased (0x%08X - 0x%08X)\r\n", APP_START_ADDR,
   //      APP_END_ADDR);

  /* 从下载区复制到 App 区 */
  uint32_t src = DOWNLOAD_START_ADDR; /* 源地址：下载区 */
  uint32_t dst = APP_START_ADDR;      /* 目标地址：App 区 */

  for (uint32_t i = 0; i < copy_size; i += 4) {
    uint32_t word = *((volatile uint32_t *)(src + i)); /* 从下载区读取 4 字节 */
    fmc_word_program(dst + i, word);                   /* 写入 App 区 */
  }
  //printf("[BL] copied %lu bytes\r\n", copy_size);

  /* 清除升级标志（擦除整页） */
  fmc_page_erase(OTA_FLAG_ADDR);
  //printf("[BL] upgrade flag cleared\r\n");

  fmc_lock();
  //printf("[BL] install complete\r\n");
}