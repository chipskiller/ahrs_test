#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GD32F3x0 OTA 协议测试脚本

【严格对照 ota_protocol.c 实现】

请求帧（上位机→设备）: [0xAA] [帧长] [功能码] [数据...] [校验码]
回复帧（设备→上位机）: [0x55] [帧长] [功能码] [数据...] [状态] [校验码]
  ★ 注意：回复帧中状态码在数据之后（与请求帧不同）

各命令帧格式（帧长 = 功能码+数据+校验码 的总字节数）:
  0xF3 查版本:  [AA][02][F3][CS]                          总4B
  0xF0 开始升级: [AA][14][F0][包数2B][版本16B][CS]          总22B
  0xF1 写入数据: [AA][00][F1][包号2B][512B数据][CS]         总518B（帧长固定0x00）
  0xF2 结束升级: [AA][03][F2][总校验码1B][CS]               总5B

校验算法: 从帧长字节开始逐字节 XOR，最后取反 (~)
总包校验码: 逐字节 XOR 累积（1字节），不取反

【使用方法】
  python ota_test.py --port COM3 --baud 9600 --firmware app.bin
  python ota_test.py --port COM3 --baud 9600 --version-only
  python ota_test.py --test-frame
  python ota_test.py --raw --port COM3 --baud 9600
"""

import sys
import os
import time
import struct
import argparse
import logging

try:
    import serial
except ImportError:
    print("[ERROR] 缺少 pyserial 库，请执行: pip install pyserial")
    sys.exit(1)

# ==================== 协议常量 ====================

OTA_HEAD = 0xAA
OTA_RSP_HEAD = 0x55

OTA_CMD_START = 0xF0
OTA_CMD_WRITE = 0xF1
OTA_CMD_FINISH = 0xF2
OTA_CMD_VERSION = 0xF3

OTA_OK = 0x00
OTA_FAIL = 0x01

OTA_PKT_DATA_SIZE = 512
OTA_VERSION_LEN = 16

DEFAULT_BAUDRATE = 9600
DEFAULT_TIMEOUT = 5.0
RESPONSE_TIMEOUT = 2.0

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger('OTA_Test')


# ==================== 帧构造（严格对照 C 代码） ====================

def calc_checksum(data: bytes) -> int:
    """
    校验码算法（与 ota_calc_checksum 完全一致）:
    逐字节 XOR，最后取反
    """
    xor_val = 0
    for b in data:
        xor_val ^= b
    return (~xor_val) & 0xFF


def build_request_frame(func_code: int, payload: bytes = b'') -> bytes:
    """
    构造请求帧（上位机→设备）

    帧格式: [0xAA] [帧长] [功能码] [数据...] [校验码]
    帧长 = 1(功能码) + len(payload) + 1(校验码)

    0xF1 写入命令特殊：帧长字段固定 0x00（C 解析器对 0xF1 做定长 518B 处理）
    """
    if func_code == OTA_CMD_WRITE:
        # 0xF1 定长帧，帧长字段固定 0x00
        length_byte = 0x00
    else:
        length_byte = (1 + len(payload) + 1) & 0xFF

    frame_body = bytes([length_byte, func_code]) + payload
    checksum = calc_checksum(frame_body)
    return bytes([OTA_HEAD]) + frame_body + bytes([checksum])


def parse_response(raw: bytes) -> dict | None:
    """
    解析设备回复帧（严格对照 ota_send_response 的输出格式）

    回复帧格式: [0x55] [帧长] [功能码] [数据...] [状态] [校验码]
    ★ 状态码在数据之后！

    帧长 = 1(功能码) + len(data) + 1(状态) + 1(校验码)
    总帧长 = 2 + 帧长
    """
    if len(raw) < 4:
        logger.warning(f"回复帧过短 ({len(raw)} bytes)")
        return None

    if raw[0] != OTA_RSP_HEAD:
        logger.warning(f"帧头错误: 期望 0x{OTA_RSP_HEAD:02X}, 收到 0x{raw[0]:02X}")
        return None

    frame_len = raw[1]
    expected_total = 2 + frame_len

    if len(raw) < expected_total:
        logger.warning(f"回复帧不完整: 期望 {expected_total} bytes, 收到 {len(raw)} bytes")
        return None

    # 校验码：从帧长字节到状态字节（不含校验码自身）
    recv_cs = raw[expected_total - 1]
    calc_cs = calc_checksum(raw[1:expected_total - 1])
    if recv_cs != calc_cs:
        logger.warning(f"校验码错误: 期望 0x{calc_cs:02X}, 收到 0x{recv_cs:02X}")
        return None

    func_code = raw[2]
    # 状态码在倒数第二个字节（校验码之前）
    status = raw[expected_total - 2]
    # 数据在功能码之后、状态码之前
    data = raw[3:expected_total - 2]

    return {
        'func_code': func_code,
        'status': status,
        'data': data
    }


def hex_dump(data: bytes, prefix: str = '') -> str:
    parts = []
    for i, b in enumerate(data):
        if i > 0 and i % 16 == 0:
            parts.append('\n' + prefix)
        parts.append(f'{b:02X} ')
    return prefix + ''.join(parts).rstrip()


# ==================== OTA 客户端 ====================

class OTAClient:
    def __init__(self, port: str, baudrate: int = DEFAULT_BAUDRATE):
        self.port = port
        self.baudrate = baudrate
        self.ser = None

    def open(self):
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=DEFAULT_TIMEOUT,
                write_timeout=DEFAULT_TIMEOUT
            )
            logger.info(f"串口已打开: {self.port} @ {self.baudrate} bps")
            return True
        except serial.SerialException as e:
            logger.error(f"串口打开失败: {e}")
            return False

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info("串口已关闭")

    def _read_exact(self, count: int, timeout: float) -> bytes | None:
        """从串口精确读取 count 字节，超时返回 None"""
        buf = bytearray()
        deadline = time.time() + timeout
        while len(buf) < count:
            remaining = count - len(buf)
            waiting = self.ser.in_waiting
            if waiting > 0:
                buf.extend(self.ser.read(min(waiting, remaining)))
            elif time.time() >= deadline:
                return None
            else:
                time.sleep(0.005)
        return bytes(buf)

    def send_and_receive(self, frame: bytes, expected_cmd: int = None,
                         timeout: float = RESPONSE_TIMEOUT) -> dict | None:
        """
        发送命令并接收回复，带完整的噪声过滤。

        采用全缓冲流式解析：所有收到的字节先存入缓冲区，
        在缓冲区中搜索有效帧。解析失败时只跳过 1 字节重新搜索，
        不会消耗属于真实帧的字节。

        参数:
            frame: 待发送的请求帧
            expected_cmd: 期望的回复功能码（用于过滤 printf 噪声）
            timeout: 超时时间（秒）
        """
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        self.ser.flush()
        logger.debug(f"TX ({len(frame)}B): {hex_dump(frame)}")

        buf = bytearray()
        deadline = time.time() + timeout
        noise_buf = bytearray()  # 用于收集噪声字节（printf 输出）

        def _flush_noise():
            """将收集的噪声字节作为 ASCII 文本显示"""
            nonlocal noise_buf
            if noise_buf:
                text = bytes(noise_buf).decode('ascii', errors='replace').rstrip('\r\n')
                if text.strip():
                    print(f"  \033[36m[DEV]\033[0m {text}")
                noise_buf.clear()

        while time.time() < deadline:
            # 读取当前所有可用字节到缓冲区
            waiting = self.ser.in_waiting
            if waiting > 0:
                buf.extend(self.ser.read(waiting))

            # 在缓冲区中搜索有效帧
            while len(buf) >= 4:  # 最小帧: [55][len][cmd+cs] = 4 bytes
                # 查找帧头 0x55
                idx = buf.find(bytes([OTA_RSP_HEAD]))
                if idx < 0:
                    # 缓冲区中没有 0x55，全部当作 printf 输出
                    noise_buf.extend(buf)
                    _flush_noise()
                    buf.clear()
                    break

                if idx > 0:
                    # 0x55 之前的字节是 printf 输出
                    noise_buf.extend(buf[:idx])
                    _flush_noise()
                    del buf[:idx]

                # 现在 buf[0] == 0x55
                if len(buf) < 2:
                    break  # 等待更多数据

                frame_len = buf[1]

                # 帧长合理性检查（最小3=cmd+status+cs，最大22=版本回复）
                if frame_len < 3 or frame_len > 22:
                    noise_buf.append(buf[0])
                    _flush_noise()
                    del buf[:1]  # 跳过这个假 0x55，继续搜索
                    continue

                expected_total = 2 + frame_len
                if len(buf) < expected_total:
                    break  # 帧不完整，等待更多数据

                # 提取候选帧
                candidate = bytes(buf[:expected_total])

                # 校验码验证
                recv_cs = candidate[-1]
                calc_cs = calc_checksum(candidate[1:-1])
                if recv_cs != calc_cs:
                    noise_buf.append(buf[0])
                    _flush_noise()
                    del buf[:1]  # 跳过这个假 0x55
                    continue

                # 功能码匹配验证
                resp_cmd = candidate[2]
                if expected_cmd is not None and resp_cmd != expected_cmd:
                    noise_buf.append(buf[0])
                    _flush_noise()
                    del buf[:1]  # 跳过这个假 0x55
                    continue

                # 所有验证通过！
                del buf[:expected_total]
                logger.debug(f"RX ({expected_total}B): {hex_dump(candidate)}")

                result = parse_response(candidate)
                return result

            # 缓冲区中没有找到有效帧，等待更多数据
            if not buf:
                time.sleep(0.01)
            else:
                # 有数据但帧不完整，短暂等待
                time.sleep(0.005)

        _flush_noise()
        logger.warning("等待回复超时")
        return None

    def query_version(self) -> str | None:
        """0xF3 查询版本"""
        logger.info("=" * 50)
        logger.info(">>> 查询版本号 (CMD=0xF3)")
        logger.info("=" * 50)

        frame = build_request_frame(OTA_CMD_VERSION)
        logger.info(f"发送帧: {hex_dump(frame, '  ')}")

        resp = self.send_and_receive(frame, expected_cmd=OTA_CMD_VERSION)
        if resp is None:
            logger.error("查询版本失败: 无回复")
            return None

        if resp['status'] != OTA_OK:
            logger.error(f"查询版本失败: 状态=0x{resp['status']:02X}")
            return None

        version = resp['data'].decode('ascii', errors='replace').rstrip('\x00')
        logger.info(f"设备固件版本: {version}")
        return version

    def start_upgrade(self, total_packets: int, version: str) -> bool:
        """
        0xF0 开始升级
        帧: [AA][14][F0][包数高][包数低][版本16B][CS]
        """
        logger.info("=" * 50)
        logger.info(">>> 开始升级 (CMD=0xF0)")
        logger.info(f"    总包数: {total_packets}")
        logger.info(f"    版本号: {version}")
        logger.info("=" * 50)

        ver_bytes = version.encode('ascii')[:OTA_VERSION_LEN]
        ver_bytes = ver_bytes.ljust(OTA_VERSION_LEN, b'\x00')
        pkt_count_bytes = struct.pack('>H', total_packets)
        payload = pkt_count_bytes + ver_bytes

        frame = build_request_frame(OTA_CMD_START, payload)
        logger.info(f"发送帧: {hex_dump(frame, '  ')}")

        resp = self.send_and_receive(frame, expected_cmd=OTA_CMD_START, timeout=5.0)
        if resp is None:
            logger.error("开始升级失败: 无回复")
            return False

        if resp['status'] != OTA_OK:
            logger.error("开始升级被拒绝: 版本相同或未知原因")
            return False

        logger.info("设备已准备就绪，下载区已擦除")
        return True

    def write_packet(self, packet_num: int, data: bytes) -> bool:
        """
        0xF1 写入数据
        帧: [AA][00][F1][包号高][包号低][512B数据][CS]
        """
        if len(data) != OTA_PKT_DATA_SIZE:
            logger.error(f"数据包长度错误: 期望 {OTA_PKT_DATA_SIZE}, 实际 {len(data)}")
            return False

        pkt_num_bytes = struct.pack('>H', packet_num)
        payload = pkt_num_bytes + data
        frame = build_request_frame(OTA_CMD_WRITE, payload)

        resp = self.send_and_receive(frame, expected_cmd=OTA_CMD_WRITE, timeout=3.0)
        if resp is None:
            logger.error(f"包 #{packet_num} 写入失败: 无回复")
            return False

        if resp['status'] != OTA_OK:
            logger.error(f"包 #{packet_num} 写入失败: 设备返回 FAIL")
            return False

        return True

    def finish_upgrade(self, total_checksum: int) -> bool:
        """
        0xF2 结束升级
        帧: [AA][03][F2][总校验码1B][CS]

        注意：C 代码中 total_checksum 是 uint8_t（1字节），
        通过逐字节 XOR 累积，不取反。
        """
        logger.info("=" * 50)
        logger.info(">>> 结束升级 (CMD=0xF2)")
        logger.info(f"    总校验码: 0x{total_checksum:02X} (1字节)")
        logger.info("=" * 50)

        # 总校验码只有 1 字节
        payload = bytes([total_checksum & 0xFF])
        frame = build_request_frame(OTA_CMD_FINISH, payload)
        logger.info(f"发送帧: {hex_dump(frame, '  ')}")

        resp = self.send_and_receive(frame, expected_cmd=OTA_CMD_FINISH, timeout=5.0)
        if resp is None:
            logger.error("结束升级失败: 无回复")
            return False

        if resp['status'] != OTA_OK:
            logger.error("结束升级失败: 设备返回 FAIL（校验不通过）")
            return False

        logger.info("升级完成！设备将重启安装新固件")
        return True

    def full_upgrade(self, firmware_path: str, target_version: str) -> bool:
        """完整 OTA 升级流程"""
        # Step 1: 查询版本
        current_version = self.query_version()
        if current_version is None:
            logger.error("无法获取设备版本，中止升级")
            return False

        if current_version == target_version:
            logger.warning(f"设备已是最新版本 ({target_version})，无需升级")
            return True

        # Step 2: 读取固件
        if not os.path.exists(firmware_path):
            logger.error(f"固件文件不存在: {firmware_path}")
            return False

        with open(firmware_path, 'rb') as f:
            firmware_data = f.read()

        file_size = len(firmware_data)
        total_packets = (file_size + OTA_PKT_DATA_SIZE - 1) // OTA_PKT_DATA_SIZE
        logger.info(f"固件文件大小: {file_size} bytes")
        logger.info(f"总包数: {total_packets} (每包 {OTA_PKT_DATA_SIZE} bytes)")

        if file_size % OTA_PKT_DATA_SIZE != 0:
            pad_size = OTA_PKT_DATA_SIZE - (file_size % OTA_PKT_DATA_SIZE)
            firmware_data += b'\xFF' * pad_size
            logger.info(f"最后一包补齐 {pad_size} bytes (0xFF)")

        # Step 3: 开始升级
        if not self.start_upgrade(total_packets, target_version):
            logger.error("开始升级失败，中止")
            return False

        # Step 4: 分包写入
        # 总校验码：逐字节 XOR 累积（1字节），与 C 代码 ota_total_checksum 一致
        total_checksum = 0
        success_count = 0
        start_time = time.time()

        for i in range(total_packets):
            pkt_data = firmware_data[i * OTA_PKT_DATA_SIZE:(i + 1) * OTA_PKT_DATA_SIZE]

            if self.write_packet(i, pkt_data):
                success_count += 1
                # 逐字节 XOR 累积（与 C 代码 ota_handle_write 中的逻辑一致）
                for b in pkt_data:
                    total_checksum ^= b
            else:
                logger.error("写入失败，中止升级")
                return False

            progress = (i + 1) / total_packets * 100
            elapsed = time.time() - start_time
            speed = (i + 1) * OTA_PKT_DATA_SIZE / elapsed / 1024 if elapsed > 0 else 0
            logger.info(f"  进度: {progress:5.1f}% ({i+1}/{total_packets}) | "
                        f"速度: {speed:.1f} KB/s")

        elapsed = time.time() - start_time
        avg_speed = file_size / elapsed / 1024 if elapsed > 0 else 0
        logger.info(f"数据发送完成: {success_count} 包成功, 耗时 {elapsed:.2f}s, "
                     f"平均速度 {avg_speed:.1f} KB/s")

        # Step 5: 结束升级
        if not self.finish_upgrade(total_checksum):
            logger.error("结束升级失败")
            return False

        logger.info("=" * 50)
        logger.info("★ OTA 升级流程全部完成！")
        logger.info("=" * 50)
        return True


# ==================== 单元测试 ====================

def test_frame_construction():
    print("\n" + "=" * 60)
    print("  OTA 协议帧构造单元测试（严格对照 C 代码）")
    print("=" * 60)

    passed = 0
    failed = 0

    # --- Test 1: 0xF3 查询版本帧 ---
    print("\n[Test 1] 查询版本帧 (0xF3)")
    print("  C代码: 帧长=2, 帧体=[F3][CS], 总4B")
    frame = build_request_frame(OTA_CMD_VERSION)
    # 帧长 = 1(功能码) + 0(数据) + 1(校验码) = 2
    expected = bytes([0xAA, 0x02, 0xF3])
    expected += bytes([calc_checksum(expected[1:])])
    if frame == expected:
        print(f"  PASS: {hex_dump(frame, '    ')}")
        passed += 1
    else:
        print(f"  FAIL: 期望 {hex_dump(expected, '    ')}")
        print(f"        实际 {hex_dump(frame, '    ')}")
        failed += 1

    # --- Test 2: 0xF0 开始升级帧 ---
    print("\n[Test 2] 开始升级帧 (0xF0)")
    print("  C代码: 帧长=0x14, 帧体=[F0][包数2B][版本16B][CS], 总22B")
    version = "AHRS_V1.0.0_2026"
    ver_bytes = version.encode('ascii').ljust(16, b'\x00')
    payload = struct.pack('>H', 100) + ver_bytes
    frame = build_request_frame(OTA_CMD_START, payload)
    # 帧长 = 1 + 18 + 1 = 20 = 0x14
    if frame[0] == OTA_HEAD and frame[1] == 0x14 and frame[2] == OTA_CMD_START:
        print(f"  PASS: 帧头=0x{frame[0]:02X}, 帧长=0x{frame[1]:02X}, "
              f"功能码=0x{frame[2]:02X}")
        print(f"        总帧长: {len(frame)} bytes")
        passed += 1
    else:
        print(f"  FAIL: 帧长=0x{frame[1]:02X} (期望 0x14)")
        failed += 1

    # --- Test 3: 0xF1 写入数据帧 ---
    print("\n[Test 3] 写入数据帧 (0xF1)")
    print("  C代码: 帧长=0x00(固定), 总518B, 定长帧")
    pkt_data = bytes(range(256)) + bytes(range(256))
    payload = struct.pack('>H', 0) + pkt_data
    frame = build_request_frame(OTA_CMD_WRITE, payload)
    if frame[0] == OTA_HEAD and frame[1] == 0x00 and frame[2] == OTA_CMD_WRITE:
        print(f"  PASS: 帧头=0x{frame[0]:02X}, 帧长=0x{frame[1]:02X}(固定), "
              f"功能码=0x{frame[2]:02X}")
        print(f"        总帧长: {len(frame)} bytes (含 512B 数据)")
        passed += 1
    else:
        print(f"  FAIL: 帧长=0x{frame[1]:02X} (期望 0x00)")
        failed += 1

    # --- Test 4: 0xF2 结束升级帧 ---
    print("\n[Test 4] 结束升级帧 (0xF2)")
    print("  C代码: 帧长=0x03, 帧体=[F2][校验码1B][CS], 总5B")
    # C 代码: uint8_t host_checksum = frame[3]; 只有 1 字节
    payload = bytes([0xCE])
    frame = build_request_frame(OTA_CMD_FINISH, payload)
    # 帧长 = 1(功能码) + 1(校验码) + 1(帧校验码) = 3 = 0x03
    if frame[0] == OTA_HEAD and frame[1] == 0x03 and frame[2] == OTA_CMD_FINISH:
        print(f"  PASS: 帧头=0x{frame[0]:02X}, 帧长=0x{frame[1]:02X}, "
              f"功能码=0x{frame[2]:02X}")
        print(f"        总帧长: {len(frame)} bytes")
        print(f"        校验码字段: 0x{frame[3]:02X} (1字节)")
        passed += 1
    else:
        print(f"  FAIL: 帧长=0x{frame[1]:02X} (期望 0x03)")
        failed += 1

    # --- Test 5: 校验码算法 ---
    print("\n[Test 5] 校验码算法验证")
    test_data = bytes([0x03, 0xF0, 0x01])
    cs = calc_checksum(test_data)
    if cs == 0x0D:
        print(f"  PASS: XOR(0x03,0xF0,0x01)=0xF2, ~0xF2=0x{cs:02X}")
        passed += 1
    else:
        print(f"  FAIL: 期望 0x0D, 实际 0x{cs:02X}")
        failed += 1

    # --- Test 6: 回复帧解析 ---
    print("\n[Test 6] 回复帧解析（状态在数据之后）")
    print("  C代码回复格式: [55][帧长][cmd][data...][status][CS]")
    # 模拟 ota_send_response(OTA_CMD_VERSION, OTA_OK, version, 16)
    # idx 变化: 0→1(帧头) →2(帧长占位) →3(cmd) →19(data 16B) →20(status) →21(CS)
    # frame_len = idx-1 = 20-1 = 19 = 0x13
    ver_resp = b'AHRS_V1.0.0_2026'
    # 构造: [55][帧长][F3][版本16B][00][CS]
    body_before_cs = bytes([OTA_CMD_VERSION]) + ver_resp + bytes([OTA_OK])
    frame_len = len(body_before_cs) + 1  # +1 for CS = 18 + 1 = 19 = 0x13
    cs_input = bytes([frame_len]) + body_before_cs
    resp_cs = calc_checksum(cs_input)
    resp_frame = bytes([OTA_RSP_HEAD, frame_len]) + body_before_cs + bytes([resp_cs])

    result = parse_response(resp_frame)
    if result and result['func_code'] == OTA_CMD_VERSION and result['status'] == OTA_OK:
        ver = result['data'].decode('ascii', errors='replace').rstrip('\x00')
        print(f"  PASS: 解析成功, 版本={ver}, 状态=0x{result['status']:02X}")
        passed += 1
    else:
        print(f"  FAIL: 解析失败 (result={result})")
        failed += 1

    # --- Test 7: 总校验码累积算法 ---
    print("\n[Test 7] 总校验码累积算法（XOR，与 C 代码一致）")
    # 模拟 C 代码: for(i=0; i<512; i++) ota_total_checksum ^= frame[5+i];
    pkt_data = bytes([0x01, 0x02, 0x03, 0x04])
    total_cs = 0
    for b in pkt_data:
        total_cs ^= b
    # 0x01 ^ 0x02 ^ 0x03 ^ 0x04 = 0x04
    if total_cs == 0x04:
        print(f"  PASS: XOR累积 = 0x{total_cs:02X}")
        passed += 1
    else:
        print(f"  FAIL: 期望 0x04, 实际 0x{total_cs:02X}")
        failed += 1

    # --- 汇总 ---
    print("\n" + "-" * 60)
    print(f"  测试结果: {passed} 通过, {failed} 失败, 共 {passed+failed} 项")
    print("-" * 60)
    return failed == 0


# ==================== 主程序 ====================

def main():
    parser = argparse.ArgumentParser(description='GD32F3x0 OTA 协议测试工具')
    parser.add_argument('--port', type=str, default='COM3', help='串口号')
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUDRATE, help=f'波特率 (默认: {DEFAULT_BAUDRATE})')
    parser.add_argument('--firmware', type=str, default=None, help='固件二进制文件路径')
    parser.add_argument('--version', type=str, default='AHRS_V2.0.0_2026', help='目标版本号')
    parser.add_argument('--version-only', action='store_true', help='仅查询版本')
    parser.add_argument('--test-frame', action='store_true', help='仅运行单元测试')
    parser.add_argument('--raw', action='store_true', help='原始监听模式')
    parser.add_argument('--raw-send', type=str, default=None, help='原始模式下先发送的hex数据')

    args = parser.parse_args()

    # 模式 0: 原始监听
    if args.raw:
        ser = serial.Serial(port=args.port, baudrate=args.baud, timeout=DEFAULT_TIMEOUT)
        logger.info(f"原始监听: {args.port} @ {args.baud} bps, Ctrl+C 退出\n")
        if args.raw_send:
            send_bytes = bytes.fromhex(args.raw_send.replace(' ', ''))
            ser.write(send_bytes)
            ser.flush()
            logger.info(f"TX: {hex_dump(send_bytes)}")
            time.sleep(0.5)
        try:
            while True:
                if ser.in_waiting > 0:
                    raw = ser.read(ser.in_waiting)
                    ts = time.strftime('%H:%M:%S')
                    hex_str = ' '.join(f'{b:02X}' for b in raw)
                    ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in raw)
                    print(f"[{ts}] RAW ({len(raw)}B): {hex_str}")
                    print(f"           ASCII: {ascii_str}")
                time.sleep(0.05)
        except KeyboardInterrupt:
            print("\n退出")
        finally:
            ser.close()
        sys.exit(0)

    # 模式 1: 单元测试
    if args.test_frame:
        success = test_frame_construction()
        sys.exit(0 if success else 1)

    # 模式 2: 仅查版本
    if args.version_only:
        client = OTAClient(args.port, args.baud)
        if not client.open():
            sys.exit(1)
        try:
            version = client.query_version()
            if version:
                print(f"\n设备固件版本: {version}")
            else:
                print("\n查询版本失败")
                sys.exit(1)
        finally:
            client.close()
        sys.exit(0)

    # 模式 3: 完整升级
    if args.firmware is None:
        print("错误: 需要 --firmware 参数")
        print("用法: python ota_test.py --firmware app.bin [--port COM3] [--baud 9600]")
        sys.exit(1)

    print("\n" + "=" * 60)
    print("  GD32F3x0 OTA 固件升级测试")
    print("=" * 60)
    print(f"  串口:     {args.port}")
    print(f"  波特率:   {args.baud}")
    print(f"  固件文件: {args.firmware}")
    print(f"  目标版本: {args.version}")
    print("=" * 60 + "\n")

    client = OTAClient(args.port, args.baud)
    if not client.open():
        sys.exit(1)

    try:
        success = client.full_upgrade(args.firmware, args.version)
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\n用户中断升级")
        sys.exit(130)
    except Exception as e:
        logger.error(f"升级异常: {e}", exc_info=True)
        sys.exit(1)
    finally:
        client.close()


if __name__ == '__main__':
    main()
