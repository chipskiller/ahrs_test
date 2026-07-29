# -*- coding: utf-8 -*-
"""
六轴传感器通信数据可视化（参考 VOFA+ 风格）

适配 communicate_protocol.h 中的协议：
    帧格式：head(0x55) | len | cmd | param | data... | checksum
    - len 表示从 len 字段本身到 data 最后一个字节的字节数
    - 总帧长 = len + 2
    - checksum = ~(buf[1] ^ buf[2] ^ ... ^ buf[len]) & 0xFF

支持功能码：
    0x82 : 角度发送        -> Roll, Pitch, Yaw, MagDisturb, Temp
    0x83 : 零位设置返回    -> ZeroSetStatus
    0x84 : 零位角度读取    -> Roll_base, Pitch_base, Yaw_base
    0x87 : 偏转报警状态    -> FaultType
    0x8A : 地磁强度读取    -> MagNorm, Mx, My, Mz
    0x8D : 主动上报 ACK    -> AckStatus
    0x8E : 心跳帧          -> Counter, IMU_Status, MAG_Status

交互说明（Windows 11 稳定版）：
    - 点击图例条目   → 切换对应通道的显示/隐藏（隐藏后半透明）
    - 拖动图例       → 可移动图例位置（matplotlib 内置 draggable）
    - 鼠标悬停绘图区：
        * 垂直虚线跟随鼠标
        * 曲线上出现圆点标记
        * 左上角信息框显示绝对时间 + 相对时间 + 各通道精确数值
    - 滚轮缩放       → 以鼠标位置为中心，同时缩放 X+Y
    - Shift + 滚轮   → 仅缩放 X 轴（时间轴）
    - Ctrl  + 滚轮   → 仅缩放 Y 轴（数值轴）
"""

import argparse
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

# ── 强制交互后端（必须在 import pyplot 之前） ──
# VSCode / Jupyter 扩展可能注入 MPLBACKEND=inline 或 agg，导致交互失效。
# 这里清除干扰并显式选择 Qt 系后端。
import os
os.environ.pop("MPLBACKEND", None)  # 清除 VSCode 可能注入的环境变量

import matplotlib
_backends_tried = []
for _be in ["QtAgg", "Qt5Agg", "TkAgg"]:
    try:
        matplotlib.use(_be, force=True)
        _backends_tried.append(_be)
        break
    except (ImportError, ModuleNotFoundError):
        _backends_tried.append(f"{_be}(不可用)")
else:
    pass  # 全部失败则使用 matplotlib 默认后端

import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg

# 设置中文字体
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "SimSun"]
plt.rcParams["axes.unicode_minus"] = False

# 渲染优化：启用路径简化，减少不可见顶点的绘制开销
plt.rcParams["path.simplify"] = True
plt.rcParams["path.simplify_threshold"] = 0.8  # 提高简化阈值，减少渲染顶点数，提升平移/缩放流畅度

# 启动时打印后端信息（方便排查 VSCode 等环境问题）
print(f"[backend] {matplotlib.get_backend()}")


# ---------------------------------------------------------------------------
# 协议解析
# ---------------------------------------------------------------------------

@dataclass
class Frame:
    timestamp: str
    rel_time: float
    cmd: int
    param: int
    data: bytes
    checksum_ok: bool
    fields: dict[str, Any] = field(default_factory=dict)


def time_str_to_seconds(time_str: str) -> float:
    """把时间字符串 HH:MM:SS.mmm 转换为当天秒数。"""
    h, m, s_ms = time_str.split(":")
    s, ms = s_ms.split(".")
    return int(h) * 3600 + int(m) * 60 + int(s) + int(ms) / 1000.0


def calc_checksum(data: bytes) -> int:
    """从 len 字段开始计算校验码：异或后取反。"""
    xor_val = 0
    for b in data:
        xor_val ^= b
    return (~xor_val) & 0xFF


def parse_signed_3byte(sign_byte: int, high_byte: int, low_byte: int) -> float:
    """解析带符号的 3 字节定点数，值本身已乘 100。"""
    val = (high_byte << 8) | low_byte
    if sign_byte == 0x80:
        val = -val
    return val / 100.0


def parse_unsigned_2byte(high_byte: int, low_byte: int) -> int:
    return (high_byte << 8) | low_byte


def parse_frame(byte_values: list[int], start: int) -> tuple[Frame | None, int]:
    """从 byte_values 的 start 位置解析一帧，返回 (Frame, next_index)。"""
    if start + 2 >= len(byte_values):
        return None, start + 1
    if byte_values[start] != 0x55:
        return None, start + 1

    length = byte_values[start + 1]
    total_len = length + 2  # head + len 内容 + checksum
    if start + total_len > len(byte_values):
        return None, start + 1

    cmd = byte_values[start + 2]
    param = byte_values[start + 3]
    data_len = max(0, length - 3)
    data = bytes(byte_values[start + 4 : start + 4 + data_len])
    checksum = byte_values[start + length + 1]
    cs_calc = calc_checksum(bytes(byte_values[start + 1 : start + 1 + length]))

    frame = Frame(
        timestamp="",
        rel_time=0.0,
        cmd=cmd,
        param=param,
        data=data,
        checksum_ok=(checksum == cs_calc),
    )
    return frame, start + total_len


def parse_cmd_fields(cmd: int, data: bytes) -> dict[str, Any]:
    """根据功能码解析数据字段。"""
    try:
        if cmd == 0x82:  # 角度发送
            if len(data) < 12:
                return {}
            return {
                "Roll": parse_signed_3byte(data[0], data[1], data[2]),
                "Pitch": parse_signed_3byte(data[3], data[4], data[5]),
                "Yaw": parse_signed_3byte(data[6], data[7], data[8]),
                "MagDisturb": data[9],
                "Temp": parse_unsigned_2byte(data[10], data[11]) / 100.0,
            }
        elif cmd == 0x83:  # 零位设置返回
            return {"ZeroSetStatus": data[0] if data else None}
        elif cmd == 0x84:  # 零位角度读取返回
            if len(data) < 9:
                return {}
            return {
                "Roll_base": parse_signed_3byte(data[0], data[1], data[2]),
                "Pitch_base": parse_signed_3byte(data[3], data[4], data[5]),
                "Yaw_base": parse_signed_3byte(data[6], data[7], data[8]),
            }
        elif cmd == 0x87:  # 偏转报警状态读取返回
            return {"FaultType": data[0] if data else None}
        elif cmd == 0x8A:  # 地磁强度读取返回
            if len(data) < 11:
                return {}
            return {
                "MagNorm": parse_unsigned_2byte(data[0], data[1]),
                "Mx": parse_signed_3byte(data[2], data[3], data[4]),
                "My": parse_signed_3byte(data[5], data[6], data[7]),
                "Mz": parse_signed_3byte(data[8], data[9], data[10]),
            }
        elif cmd == 0x8D:  # 主动上报 ACK
            return {"AckStatus": data[0] if data else None}
        elif cmd == 0x8E:  # 心跳帧
            if len(data) < 3:
                return {}
            return {
                "Counter": data[0],
                "IMU_Status": data[1],
                "MAG_Status": data[2],
            }
        else:
            return {"Raw": data.hex()}
    except Exception:
        return {"Raw": data.hex()}


def parse_file(file_path: str | Path) -> list[Frame]:
    """解析单个 txt 文件，返回所有帧列表。"""
    file_path = Path(file_path)
    if not file_path.exists():
        raise FileNotFoundError(f"找不到文件: {file_path}")

    line_pattern = re.compile(
        r"\[(\d{2}:\d{2}:\d{2}\.\d{3})\]收←◆((?:[0-9A-Fa-f]{2}\s*)+)"
    )

    records: list[Frame] = []
    day_offset = 0.0
    prev_seconds: float | None = None
    base_seconds: float | None = None

    with open(file_path, "r", encoding="utf-8") as f:
        for line in f:
            match = line_pattern.search(line)
            if not match:
                continue

            time_str, hex_str = match.groups()
            current_seconds = time_str_to_seconds(time_str)

            if prev_seconds is not None and current_seconds < prev_seconds - 12 * 3600:
                day_offset += 24 * 3600
            corrected_seconds = current_seconds + day_offset

            if base_seconds is None:
                base_seconds = corrected_seconds
            prev_seconds = current_seconds

            byte_values = [int(b, 16) for b in hex_str.split()]
            i = 0
            while i < len(byte_values):
                frame, next_i = parse_frame(byte_values, i)
                if frame is None:
                    i += 1
                else:
                    frame.timestamp = time_str
                    frame.rel_time = corrected_seconds - base_seconds
                    frame.fields = parse_cmd_fields(frame.cmd, frame.data)
                    records.append(frame)
                    i = next_i

    return records


# ---------------------------------------------------------------------------
# 交互式可视化
# ---------------------------------------------------------------------------

CMD_NAMES = {
    0x82: "角度发送 (0x82)",
    0x83: "零位设置返回 (0x83)",
    0x84: "零位角度读取 (0x84)",
    0x87: "偏转报警状态 (0x87)",
    0x8A: "地磁强度读取 (0x8A)",
    0x8D: "主动上报 ACK (0x8D)",
    0x8E: "心跳帧 (0x8E)",
}

# 使用 Set1 + Set2 组合色板，确保相邻通道颜色明显不同
# Set1: 红、橙、蓝、绿、紫、棕、粉、灰、橄榄
# Set2: 蓝绿、橙、灰紫、黄绿、玫瑰、天蓝、橄榄绿、紫
_set1 = plt.cm.Set1(np.linspace(0, 1, 9))
_set2 = plt.cm.Set2(np.linspace(0, 1, 8))
COLORS = np.vstack([_set1, _set2])  # 共 17 种明显不同的颜色


_qt_windows = []  # 保持窗口引用，防止垃圾回收


class PureQtWindow:
    """纯 Qt 窗口，直接包装 FigureCanvasQTAgg。

    完全绕过 pyplot 的 FigureManager 和 plt.show()，
    避免 VSCode 环境下事件循环不阻塞的问题。
    """

    def __init__(self, fig, figsize=(14, 6), title=""):
        from PyQt5.QtWidgets import QMainWindow, QWidget, QVBoxLayout
        from PyQt5.QtCore import Qt

        self.fig = fig
        self.canvas = fig.canvas  # 复用 build_figure 中已创建的 FigureCanvasQTAgg
        self.ax = fig.axes[0] if fig.axes else None
        self.viewer = None  # 由外部赋值

        # 防止 Qt 在绘制前清除背景（消除 Windows 11 下的初始残影）
        self.canvas.setAttribute(Qt.WA_OpaquePaintEvent, True)

        # 主窗口
        self.window = QMainWindow()
        central = QWidget()
        layout = QVBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.canvas)
        self.window.setCentralWidget(central)
        self.window.resize(int(figsize[0] * 100), int(figsize[1] * 100))
        self.window.setWindowTitle(title)

    def show(self):
        self.window.show()
        self.window.raise_()
        self.window.activateWindow()
        # 窗口显示后强制重绘并刷新 Qt 绘制管线，确保 canvas 在正确 DPI 下渲染干净画面。
        # processEvents 确保 paint 事件立即处理，避免 blit 背景捕获到初始化残留。
        self.fig.canvas.draw()
        from PyQt5.QtWidgets import QApplication
        app = QApplication.instance()
        if app is not None:
            app.processEvents()


class VOFAViewer:
    """VOFA 风格交互式查看器（Windows 11 稳定版）。

    交互方式：
    - 左键点击图例  → 切换通道显示/隐藏  (button_press_event)
    - 滚轮          → 缩放 X+Y 轴       (scroll_event)
    - Shift + 滚轮  → 仅缩放 X 轴（时间轴）
    - Ctrl  + 滚轮  → 仅缩放 Y 轴（数值轴）
    - 悬停          → 显示绝对时间 + 相对时间 + 各通道数值
    - 右键拖拽      → 平移               (matplotlib 内置)
    """

    def __init__(self, fig, ax, records: list[Frame],
                 lines: dict[str, plt.Line2D], field_names: list[str],
                 legend):
        self.fig = fig
        self.ax = ax
        self.records = records
        self.lines = lines
        self.field_names = field_names
        self.legend = legend
        self.x = np.array([r.rel_time for r in records])
        self.timestamps = [r.timestamp for r in records]  # 绝对时间戳
        self.y = {
            name: np.array([r.fields.get(name) for r in records], dtype=float)
            for name in lines
        }

        # --- 图例点击切换：建立 legend line → data line 映射 ---
        self.lined = {}
        for legline, name in zip(legend.get_lines(), field_names):
            self.lined[legline] = lines[name]

        # --- 数据光标元素（animated=True 使 draw() 跳过它们，加速缩放/平移重绘） ---
        self.vline = ax.axvline(
            x=0, color="dimgray", linestyle="--", alpha=0.6,
            visible=False, zorder=99, linewidth=1, animated=True,
        )
        self.markers = {}
        for name, line in lines.items():
            marker, = ax.plot(
                [], [], 'o', color=line.get_color(), markersize=7,
                markerfacecolor=line.get_color(), markeredgecolor='white',
                markeredgewidth=1.5, visible=False, zorder=100, animated=True,
            )
            self.markers[name] = marker

        # 左上角信息框（使用 Microsoft YaHei 以支持中文"时间"标签）
        self.info_text = ax.text(
            0.02, 0.98, "",
            transform=ax.transAxes,
            verticalalignment='top',
            horizontalalignment='left',
            bbox=dict(boxstyle="round,pad=0.4", fc="lightyellow", alpha=0.95, ec="gray"),
            fontsize=9,
            fontfamily=['Microsoft YaHei', 'Consolas', 'Courier New', 'monospace'],
            visible=False,
            zorder=101,
            animated=True,
        )

        # --- 右键拖拽平移状态 ---
        self._pan_start_x = None
        self._pan_start_y = None
        self._pan_xlim = None
        self._pan_ylim = None

        # --- Blitting 状态（只重绘光标元素，不重绘整张图，大幅提升帧率） ---
        self._blit_bg = None
        self._blit_init = False
        self._blit_invalid = True  # 背景需要重捕（视图变化 / 图例拖动后为 True）

        # --- 连接事件 ---
        cid1 = self.fig.canvas.mpl_connect("button_press_event", self._on_click)
        cid2 = self.fig.canvas.mpl_connect("motion_notify_event", self._on_move)
        cid3 = self.fig.canvas.mpl_connect("scroll_event", self._on_scroll)
        cid4 = self.fig.canvas.mpl_connect("button_release_event", self._on_release)
        cid5 = self.fig.canvas.mpl_connect("resize_event", self._on_resize)
        print(f"  [EVENT] Connected: click={cid1}, move={cid2}, scroll={cid3}, "
              f"release={cid4}, resize={cid5}", flush=True)

        # 图例拖动时标记背景失效（draggable 内部用 _on_offset_change）
        try:
            orig = legend._on_offset_change

            def _legend_moved(*a, **kw):
                orig(*a, **kw)
                self._blit_invalid = True

            legend._on_offset_change = _legend_moved
        except Exception:
            pass

    def _hide_cursor(self):
        """隐藏所有光标元素（防止缩放/平移后留下残影）。"""
        self.vline.set_visible(False)
        for m in self.markers.values():
            m.set_visible(False)
        self.info_text.set_visible(False)

    def _get_data_coords(self, event):
        """获取鼠标在 self.ax 中的数据坐标 (xdata, ydata)。

        优先使用 event.xdata/ydata；若为 None（某些环境下 inaxes 检测失败），
        则通过 display 坐标手动转换。返回 (xdata, ydata) 或 (None, None)。
        """
        if event.xdata is not None and event.ydata is not None:
            return event.xdata, event.ydata
        # 回退：用 display 坐标手动转换（不做 bbox 检查，DPI 缩放会导致误判）
        if event.x is None or event.y is None:
            return None, None
        try:
            xdata, ydata = self.ax.transData.inverted().transform((event.x, event.y))
            xdata, ydata = float(xdata), float(ydata)
            # 检查转换结果是否有效（非 NaN/Inf）
            if not (np.isfinite(xdata) and np.isfinite(ydata)):
                return None, None
            return xdata, ydata
        except Exception:
            return None, None

    # ---- 图例点击检测（通过 button_press_event 手动判断） ----
    def _on_legend_click(self, event):
        """检测鼠标点击了哪个图例条目，返回条目索引或 None。

        原理：利用 legend.get_texts() 获取各条目文本的位置，
        根据点击的 y 坐标判断点击了哪一行。
        legend.get_texts() 和 legend.get_lines() 顺序一致。
        """
        legend = self.legend
        if legend is None:
            return None

        # 获取图例整体包围盒（display 坐标）
        bbox = legend.get_window_extent()
        if not bbox.contains(event.x, event.y):
            return None

        texts = legend.get_texts()
        if not texts:
            return None

        # 优先：用各 Text 的 y 范围精确判断点击行
        try:
            for i, text in enumerate(texts):
                tb = text.get_window_extent()
                # y 在该文本范围内，且 x 在图例水平范围内
                if (tb.y0 - 4 <= event.y <= tb.y1 + 4
                        and bbox.x0 <= event.x <= bbox.x1):
                    return i
        except Exception:
            pass

        # 回退：按图例总高度均分行高
        n = len(texts)
        row_h = bbox.height / n if n > 0 else bbox.height
        if row_h > 0:
            idx = int((bbox.y1 - event.y) / row_h)
            if 0 <= idx < n:
                return idx

        return None

    # ---- 鼠标点击：处理图例点击切换通道 + 右键拖拽平移起始 ----
    def _on_click(self, event):
        """左键点击图例条目时切换对应曲线的显示/隐藏；右键开始平移。"""
        if event.button == 3:
            # 右键 → 记录平移起始位置
            self._pan_start_x = event.x
            self._pan_start_y = event.y
            self._pan_xlim = self.ax.get_xlim()
            self._pan_ylim = self.ax.get_ylim()
            # 预计算像素→数据坐标的缩放因子（避免每次 motion 做昂贵的逆变换）
            w = self._pan_xlim[1] - self._pan_xlim[0]
            h = self._pan_ylim[1] - self._pan_ylim[0]
            bbox = self.ax.get_window_extent()
            self._pan_sx = w / max(bbox.width, 1)
            self._pan_sy = h / max(bbox.height, 1)
            # 平移开始时立即隐藏光标，避免后续 draw_idle 捕获带光标的背景
            self._hide_cursor()
            self._blit_invalid = True
            # 临时提高路径简化阈值 → 渲染顶点更少 → draw_idle 更快
            self._old_simplify = matplotlib.rcParams["path.simplify_threshold"]
            matplotlib.rcParams["path.simplify_threshold"] = 1.0
            return
        if event.button != 1:  # 仅左键
            return

        # 先检测是否点击了图例（图例可能与绘图区重叠，需优先判断）
        idx = self._on_legend_click(event)
        if idx is not None:
            name = self.field_names[idx]
            line = self.lines[name]
            legline = list(self.lined.keys())[idx]
            vis = not line.get_visible()
            line.set_visible(vis)
            legline.set_alpha(1.0 if vis else 0.3)
            self.fig.canvas.draw()
            self._blit_invalid = True  # 曲线可见性变化 → 背景需重捕
            return

    # ---- Blitting：只重绘光标元素，不重绘整张图（大幅提升帧率） ----
    def _setup_blit(self):
        """首次调用时捕获静态背景（曲线/网格/图例），后续 blit 只覆盖光标。"""
        self._hide_cursor()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()  # 同步刷新，确保屏幕与缓冲区一致
        self._blit_bg = self.fig.canvas.copy_from_bbox(self.ax.bbox)
        self._blit_init = True
        self._blit_invalid = False

    # ---- 鼠标移动：数据光标（blitting 加速） ----
    def _on_move(self, event):
        # --- 右键拖拽平移（预计算缩放因子 + 高路径简化 → 快速 draw_idle） ---
        if (self._pan_start_x is not None
                and event.x is not None and event.y is not None
                and event.button == 3):
            ddx = (event.x - self._pan_start_x) * self._pan_sx
            ddy = (event.y - self._pan_start_y) * self._pan_sy
            self.ax.set_xlim(self._pan_xlim[0] - ddx, self._pan_xlim[1] - ddx)
            self.ax.set_ylim(self._pan_ylim[0] - ddy, self._pan_ylim[1] - ddy)
            self.fig.canvas.draw_idle()
            return

        # 延迟初始化 blitting（确保 canvas 已渲染）
        if not self._blit_init:
            self._setup_blit()

        x_mouse, y_mouse = self._get_data_coords(event)

        if x_mouse is None:
            if self.vline.get_visible():
                self.vline.set_visible(False)
                for m in self.markers.values():
                    m.set_visible(False)
                self.info_text.set_visible(False)
                # 背景失效时先重捕
                if self._blit_invalid:
                    self._setup_blit()
                self.fig.canvas.restore_region(self._blit_bg)
                self.ax.draw_artist(self.vline)  # 隐藏状态下不绘制，但安全起见
                self.fig.canvas.blit(self.ax.bbox)
            return

        # O(log n) 查找最近数据点（x 数组单调递增）
        idx = int(np.searchsorted(self.x, x_mouse))
        idx = max(0, min(idx, len(self.x) - 1))
        # 检查前一个点是否更近
        if idx > 0 and abs(self.x[idx - 1] - x_mouse) < abs(self.x[idx] - x_mouse):
            idx -= 1
        x_val = self.x[idx]

        visible_values = []
        for name in self.field_names:
            line = self.lines[name]
            if not line.get_visible():
                self.markers[name].set_visible(False)
                continue
            y_val = self.y[name][idx]
            if np.isnan(y_val):
                self.markers[name].set_visible(False)
                continue
            visible_values.append((name, y_val))
            self.markers[name].set_data([x_val], [y_val])
            self.markers[name].set_visible(True)

        self.vline.set_xdata([x_val, x_val])
        self.vline.set_visible(True)

        if not visible_values:
            self.info_text.set_visible(False)
        else:
            abs_time = self.timestamps[idx] if idx < len(self.timestamps) else "??:??:??.???"
            parts = [
                f"时间  {abs_time}",
                f"t = {x_val:.3f} s",
                "-" * 24,
            ]
            for name, y_val in visible_values:
                parts.append(f"  {name:12s}: {y_val:>10.4f}")
            self.info_text.set_text("\n".join(parts))
            self.info_text.set_visible(True)

        # --- Blitting：恢复背景 → 绘制动画元素 → blit ---
        if self._blit_invalid:
            self._setup_blit()
        self.fig.canvas.restore_region(self._blit_bg)
        if self.vline.get_visible():
            self.ax.draw_artist(self.vline)
        for m in self.markers.values():
            if m.get_visible():
                self.ax.draw_artist(m)
        if self.info_text.get_visible():
            self.ax.draw_artist(self.info_text)
        self.fig.canvas.blit(self.ax.bbox)

    # ---- 滚轮缩放（支持修饰键独立缩放 X/Y 轴） ----
    def _on_scroll(self, event):
        """滚轮缩放：以鼠标位置为中心放大/缩小。

        - 无修饰键：同时缩放 X 和 Y
        - Shift + 滚轮：仅缩放 X 轴（时间轴）
        - Ctrl  + 滚轮：仅缩放 Y 轴（数值轴）
        """
        ax = self.ax
        curx, cury = self._get_data_coords(event)

        # 检测修饰键：优先 event.key，后备 Qt 直接查询（某些后端 event.key 为 None）
        mod_key = event.key
        if mod_key is None:
            try:
                from PyQt5.QtCore import Qt
                from PyQt5.QtWidgets import QApplication
                qmods = QApplication.keyboardModifiers()
                if qmods & Qt.ShiftModifier:
                    mod_key = 'shift'
                elif qmods & Qt.ControlModifier:
                    mod_key = 'control'
            except Exception:
                pass

        if curx is None or cury is None:
            return

        x_min, x_max = ax.get_xlim()
        y_min, y_max = ax.get_ylim()
        w = x_max - x_min
        h = y_max - y_min

        # 缩放因子
        factor = 1.15 if event.button == 'down' else 1 / 1.15

        # 根据修饰键决定缩放哪个轴
        zoom_x = True
        zoom_y = True
        if mod_key == 'shift':
            zoom_y = False   # Shift → 仅 X 轴
        elif mod_key == 'control':
            zoom_x = False   # Ctrl  → 仅 Y 轴

        if zoom_x:
            w = w * factor
        if zoom_y:
            h = h * factor

        # 以鼠标位置为缩放中心（curx/cury 已从 _get_data_coords 获取）
        curXpos = (curx - x_min) / (x_max - x_min) if (x_max - x_min) != 0 else 0.5
        curYpos = (cury - y_min) / (y_max - y_min) if (y_max - y_min) != 0 else 0.5

        if zoom_x:
            newx = curx - w * curXpos
            ax.set_xlim(newx, newx + w)
        if zoom_y:
            newy = cury - h * curYpos
            ax.set_ylim(newy, newy + h)
        self._hide_cursor()
        self._blit_invalid = True  # 视图变化 → 背景需重捕
        self.fig.canvas.draw_idle()

    # ---- 鼠标释放：结束平移 ----
    def _on_release(self, event):
        """右键释放时结束平移。"""
        if event.button == 3:
            self._pan_start_x = None
            self._pan_start_y = None
            self._pan_xlim = None
            self._pan_ylim = None
            # 恢复路径简化阈值
            if hasattr(self, '_old_simplify'):
                matplotlib.rcParams["path.simplify_threshold"] = self._old_simplify
            self._blit_invalid = True  # 平移结束 → 背景需重捕
            self.fig.canvas.draw_idle()  # 触发全图重绘，清理 blit 残留

    # ---- 窗口大小调整：清理 blit 状态，防止残影 ----
    def _on_resize(self, event):
        """窗口大小变化时重置 blit 状态，强制下次交互重捕背景。"""
        self._blit_invalid = True
        self._blit_init = False  # 强制重新初始化 blit 背景
        self._hide_cursor()


def _decimate_for_display(x: np.ndarray, y: np.ndarray,
                          target: int = 1500) -> tuple[np.ndarray, np.ndarray]:
    """Min-max 抽取：将数据降至 target 个点，保留每个桶的极值。

    视觉几乎无损（屏幕仅 ~1400 像素），但渲染顶点数大幅减少。
    数据量 <= target*2 时直接返回原数组。
    """
    n = len(x)
    if n <= target * 2:
        return x, y
    bucket = max(1, n // target)
    idx_parts = []
    for i in range(0, n, bucket):
        j = min(i + bucket, n)
        seg = slice(i, j)
        seg_y = y[seg]
        local = np.arange(j - i)
        idx_parts.append(i + local[np.argmin(seg_y)])
        idx_parts.append(i + local[np.argmax(seg_y)])
    idx = np.unique(np.sort(np.array(idx_parts, dtype=int)))
    return x[idx], y[idx]


def build_figure(records: list[Frame], cmd: int, file_name: str = "") -> tuple[plt.Figure, dict] | None:
    """创建交互式波形图。返回 (fig, info_dict) 或 None。"""
    cmd_records = [r for r in records if r.cmd == cmd]
    if not cmd_records:
        print(f"未找到功能码 0x{cmd:02X} 的数据")
        return None

    field_names = []
    for r in cmd_records:
        for name in r.fields:
            if name not in field_names:
                field_names.append(name)

    if not field_names:
        print(f"功能码 0x{cmd:02X} 没有可解析字段")
        return None

    # --- 纯 Qt 图形（绕过 pyplot 的 FigureManager） ---
    fig = Figure(figsize=(14, 6))
    canvas = FigureCanvasQTAgg(fig)  # 必须先于 VOFAViewer 创建，事件连接依赖 canvas
    ax = fig.add_subplot(111)

    # --- 绘制曲线（显示数据经 min-max 抽取，渲染更快） ---
    x_full = np.array([r.rel_time for r in cmd_records])
    lines: dict[str, plt.Line2D] = {}
    for idx, name in enumerate(field_names):
        y_full = np.array([r.fields.get(name) for r in cmd_records], dtype=float)
        x_disp, y_disp = _decimate_for_display(x_full, y_full)
        color = COLORS[idx % len(COLORS)]
        line, = ax.plot(x_disp, y_disp, label=name, color=color, linewidth=1.3)
        lines[name] = line

    # --- 图例（可点击切换，可拖动移动位置） ---
    legend = ax.legend(
        loc='upper right',
        fontsize=9,
        framealpha=0.9,
        shadow=True,
        fancybox=True,
    )
    # 让图例可拖动（matplotlib 内置功能）
    if hasattr(legend, 'set_draggable'):
        legend.set_draggable(True)

    # --- 主图样式 ---
    title = (
        f"{file_name}  |  {CMD_NAMES.get(cmd, f'功能码 0x{cmd:02X}')}  —  共 {len(cmd_records)} 帧\n"
        "滚轮缩放XY | Shift+滚轮缩放X | Ctrl+滚轮缩放Y | 点击图例切换通道 | 悬停看坐标"
    )
    ax.set_title(title, fontsize=10, pad=10)
    ax.set_xlabel("相对时间 (s)", fontsize=11)
    ax.set_ylabel("数值", fontsize=11)
    ax.grid(True, linestyle="--", alpha=0.6)

    # --- 创建交互控制器 ---
    viewer = VOFAViewer(fig, ax, cmd_records, lines, field_names, legend)

    return fig, {"viewer": viewer, "lines": lines, "legend": legend}


def _show_figures_qt():
    """启动 Qt 事件循环，显示所有已创建的 PureQtWindow 窗口。

    完全绕过 plt.show()，直接管理 QApplication 和窗口生命周期。
    这是解决 VSCode 环境下 plt.show() 不阻塞的唯一可靠方法。
    """
    from PyQt5.QtWidgets import QApplication
    import sys

    if not _qt_windows:
        print("  [QT] 没有窗口可显示", flush=True)
        return

    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)

    for pw in _qt_windows:
        pw.show()

    print(f"  [QT] 事件循环启动，共 {len(_qt_windows)} 个窗口", flush=True)
    app.exec_()
    print("  [QT] 事件循环结束", flush=True)

    _qt_windows.clear()


def plot_cmd(records: list[Frame], cmd: int, file_name: str = "", show: bool = True) -> plt.Figure | None:
    """为指定功能码绘制一个 VOFA 风格的波形图。"""
    result = build_figure(records, cmd, file_name)
    if result is None:
        return None
    fig, info = result

    # savefig 在 show 之前（show 会阻塞直到窗口关闭）
    safe_name = file_name.replace(".", "_").replace(" ", "_")
    png_name = f"{safe_name}_0x{cmd:02X}_{CMD_NAMES.get(cmd, 'cmd').split(' ')[0]}.png"
    try:
        fig.savefig(png_name, dpi=300, bbox_inches="tight")
    except Exception:
        pass

    if show:
        title = f"{file_name} | {CMD_NAMES.get(cmd, f'0x{cmd:02X}')} — {len([r for r in records if r.cmd == cmd])} 帧"
        pw = PureQtWindow(fig, title=title)
        pw.viewer = info["viewer"]
        _qt_windows.append(pw)  # 保持引用
        pw.show()

    return fig


def plot_all(records: list[Frame], file_name: str = "") -> None:
    """为文件中出现的所有功能码分别创建图形窗口。"""
    cmds = sorted(set(r.cmd for r in records))
    for cmd in cmds:
        plot_cmd(records, cmd, file_name, show=True)
    _show_figures_qt()


# ---------------------------------------------------------------------------
# 数据导出
# ---------------------------------------------------------------------------

def export_data(records: list[Frame], output_dir: str | Path | None = None) -> None:
    """按功能码导出 CSV 和 Excel。"""
    import pandas as pd

    output_dir = Path(output_dir) if output_dir else Path.cwd()
    output_dir.mkdir(parents=True, exist_ok=True)

    cmds = sorted(set(r.cmd for r in records))
    for cmd in cmds:
        cmd_records = [r for r in records if r.cmd == cmd]
        if not cmd_records:
            continue

        rows = []
        for r in cmd_records:
            row = {
                "时间戳": r.timestamp,
                "绝对时间": r.timestamp,
                "相对时间(s)": round(r.rel_time, 3),
                "功能码": f"0x{r.cmd:02X}",
                "参数": f"0x{r.param:02X}",
                "校验通过": r.checksum_ok,
            }
            row.update(r.fields)
            rows.append(row)

        df = pd.DataFrame(rows)
        cmd_str = f"0x{cmd:02X}"
        csv_path = output_dir / f"功能码_{cmd_str}_数据.csv"
        xlsx_path = output_dir / f"功能码_{cmd_str}_数据.xlsx"

        df.to_csv(csv_path, index=False, encoding="utf-8-sig")
        df.to_excel(xlsx_path, index=False, engine="openpyxl")
        print(f"已导出: {csv_path}")
        print(f"已导出: {xlsx_path}")


# ---------------------------------------------------------------------------
# 主程序
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(description="六轴传感器通信数据可视化（VOFA 风格）")
    parser.add_argument(
        "files",
        nargs="*",
        help="一个或多个数据 txt 文件路径",
    )
    parser.add_argument(
        "--cmd",
        type=str,
        default=None,
        help="只查看指定功能码，例如 0x82、82、angle、heartbeat 等",
    )
    parser.add_argument(
        "--export",
        action="store_true",
        help="仅导出 CSV/Excel，不绘图",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="导出目录，默认为当前工作目录",
    )
    parser.add_argument(
        "--list-cmds",
        action="store_true",
        help="只列出文件中出现的功能码及其帧数，不绘图",
    )
    args = parser.parse_args()

    if args.files:
        data_files = [Path(f) for f in args.files]
    else:
        data_files = [
            Path(r"c:\Users\EDY\.trae-cn\attachments\6a66cfabfcd611c6f3a58fb5\09bd945a-aa31-4f23-ae5c-07ac32b7857b_ba93d533-a00e-450d-b61d-68b1b89a39be_7.29.txt"),
        ]

    # 解析所有文件
    all_records: list[Frame] = []
    for f in data_files:
        if not f.exists():
            print(f"警告: 未找到 {f}")
            continue
        print(f"正在解析: {f.name}")
        all_records.extend(parse_file(f))

    if not all_records:
        print("没有解析到任何数据")
        return

    print(f"共解析 {len(all_records)} 帧")

    if args.list_cmds:
        cmds = sorted(set(r.cmd for r in all_records))
        print("\n文件中出现的功能码：")
        for cmd in cmds:
            count = sum(1 for r in all_records if r.cmd == cmd)
            print(f"  0x{cmd:02X} : {CMD_NAMES.get(cmd, '未知')} - {count} 帧")
        return

    # 解析 --cmd 参数（导出与绘图共用）
    target_cmd: int | None = None
    if args.cmd:
        cmd_lower = args.cmd.lower()
        alias_map = {
            "angle": 0x82,
            "zero_set": 0x83,
            "zero": 0x84,
            "alarm": 0x87,
            "mag": 0x8A,
            "ack": 0x8D,
            "heartbeat": 0x8E,
        }
        if cmd_lower in alias_map:
            target_cmd = alias_map[cmd_lower]
        elif args.cmd.startswith("0x"):
            target_cmd = int(args.cmd, 16)
        else:
            target_cmd = int(args.cmd, 16)

    # 导出
    export_records = all_records
    if target_cmd is not None:
        export_records = [r for r in all_records if r.cmd == target_cmd]
    if args.export:
        print("开始导出...")
        export_data(export_records, args.output_dir)
        return

    # 绘图
    if target_cmd is not None:
        plot_cmd(all_records, target_cmd, "合并数据")
    else:
        plot_all(all_records, "合并数据")

    print("导出数据表...")
    export_data(export_records, args.output_dir)


if __name__ == "__main__":
    main()
