# AHRS 传感器通信数据可视化工具

解析六轴传感器（AHRS）串口通信日志，自动识别协议帧并生成 VOFA+ 风格的交互式波形图，同时支持数据导出为 CSV/Excel。

## 功能概览

- **协议自动解析** — 支持 0x55 帧头、可变长度帧（0x0D/0x0E/0x0F），自动校验
- **多功能码识别** — 角度(0x82)、零位设置(0x83)、零位读取(0x84)、报警(0x87)、地磁(0x8A)、ACK(0x8D)、心跳(0x8E)
- **交互式波形图** — 图例点击切换通道、滚轮缩放、悬停数据光标、绝对时间显示
- **数据导出** — 按功能码分组导出 CSV + Excel，包含绝对时间、相对时间和解码后的数值

## 环境要求

| 依赖 | 版本 | 用途 |
|---|---|---|
| Python | 3.11+ | 运行环境 |
| matplotlib | 3.10+ | 绘图 + 交互 |
| numpy | 2.2+ | 数值计算 |
| pandas | 2.3+ | 数据导出 |
| openpyxl | 3.1+ | Excel 导出引擎 |
| PyQt5 | 5.15+ | Qt 交互后端（**必须**，否则无法交互） |

安装依赖：

```bash
pip install matplotlib numpy pandas openpyxl PyQt5
```

> **注意**：PyQt5 是交互功能的关键依赖。缺少它会导致默认后端为非交互式的 `agg`，无法弹出窗口、无法响应鼠标事件。

## 数据文件格式

程序解析的串口日志格式如下，每行包含时间戳和十六进制数据：

```
[10:23:45.123]收←◆55 0D 82 00 80 00 64 00 00 00 0A 0F 00 73
[10:23:45.234]收←◆55 0D 82 00 80 00 C8 00 00 00 0A 0F 00 23
```

- 时间戳格式：`HH:MM:SS.mmm`
- 数据格式：空格分隔的十六进制字节
- 帧头：`0x55`

## 使用方法

### 基本用法

```bash
# 解析并可视化默认数据文件
python visualize_6axis.py

# 指定数据文件
python visualize_6axis.py data.txt

# 多个文件合并分析
python visualize_6axis.py file1.txt file2.txt
```

### 命令行参数

```bash
# 只查看角度数据（功能码 0x82）
python visualize_6axis.py --cmd 0x82 data.txt
python visualize_6axis.py --cmd angle data.txt

# 只查看地磁数据
python visualize_6axis.py --cmd 0x8A data.txt
python visualize_6axis.py --cmd mag data.txt

# 只查看心跳数据
python visualize_6axis.py --cmd heartbeat data.txt

# 仅导出数据，不绘图
python visualize_6axis.py --export data.txt

# 导出到指定目录
python visualize_6axis.py --export --output-dir ./output data.txt

# 列出文件中所有功能码及帧数
python visualize_6axis.py --list-cmds data.txt
```

### 功能码别名

| 别名 | 功能码 | 说明 |
|---|---|---|
| `angle` | 0x82 | 角度发送（Roll, Pitch, Yaw, MagDisturb[, Temp]） |
| `zero_set` | 0x83 | 零位设置返回 |
| `zero` | 0x84 | 零位角度读取 |
| `alarm` | 0x87 | 偏转报警状态 |
| `mag` | 0x8A | 地磁强度读取（MagNorm, Mx, My, Mz） |
| `ack` | 0x8D | 主动上报 ACK |
| `heartbeat` | 0x8E | 心跳帧（Counter, IMU_Status, MAG_Status） |

## 交互操作

图形窗口支持以下交互操作：

| 操作 | 功能 |
|---|---|
| **左键点击图例** | 切换对应通道的显示/隐藏（隐藏后图例半透明） |
| **拖动图例** | 移动图例位置，避免遮挡曲线 |
| **滚轮** | 以鼠标位置为中心，同时缩放 X+Y 轴 |
| **Shift + 滚轮** | 仅缩放 X 轴（时间轴） |
| **Ctrl + 滚轮** | 仅缩放 Y 轴（数值轴） |
| **右键拖拽** | 平移视图 |
| **鼠标悬停** | 显示数据光标：垂直虚线 + 圆点标记 + 信息框 |

### 信息框内容

悬停时左上角信息框显示：

```
时间  10:23:45.123        ← 绝对时间（来自串口日志时间戳）
t = 0.000 s               ← 相对时间（距首帧的秒数）
------------------------
  Roll       :     1.0000  ← 各通道精确数值（4 位小数）
  Pitch      :    -2.0000
  Yaw        :     0.0000
```

## 协议说明

### 帧格式

```
head(0x55) | len | cmd | param | data... | checksum
```

- `len`：从 `len` 字段本身到 `data` 最后一个字节的字节数
- 总帧长 = `len + 2`（加上 `head` 和 `checksum`）
- `checksum = ~(buf[1] ^ buf[2] ^ ... ^ buf[len]) & 0xFF`

### 支持的功能码

| 功能码 | 名称 | 数据字段 | 帧长 |
|---|---|---|---|
| 0x82 | 角度发送 | Roll, Pitch, Yaw, MagDisturb[, Temp] | 15 或 13 字节 |
| 0x83 | 零位设置返回 | ZeroSetStatus | 14 字节 |
| 0x84 | 零位角度读取 | Roll_base, Pitch_base, Yaw_base | 17 字节 |
| 0x87 | 偏转报警状态 | FaultType | 14 字节 |
| 0x8A | 地磁强度读取 | MagNorm, Mx, My, Mz | 17 字节 |
| 0x8D | 主动上报 ACK | AckStatus | 14 字节 |
| 0x8E | 心跳帧 | Counter, IMU_Status, MAG_Status | 15 字节 |

> **注**：0x82 的温度字段（Temp）是可选项。`communicate_protocol.h` 中固件若将温度字节注释掉，单帧 0x82 帧长为 13 字节（data=10 字节）；含温度时为 15 字节（data=12 字节）。解析器会按帧长自动识别，含温度则输出 Temp 列，不含则跳过（可视化时 Temp 曲线留空）。

### 数据编码

- **带符号 3 字节**：`[sign_byte, high_byte, low_byte]`，值 = `(high<<8 | low) / 100.0`，`sign_byte=0x80` 时为负
- **无符号 2 字节**：`(high_byte << 8) | low_byte`
- **IMU 温度**：2 字节（高字节在前），值 = `(high<<8 | low) / 100.0`（单位 °C）

## 导出格式

数据按功能码分组导出，每个功能码生成一个 CSV 和一个 Excel 文件：

```
功能码_0x82_数据.csv
功能码_0x82_数据.xlsx
功能码_0x8A_数据.csv
功能码_0x8A_数据.xlsx
...
```

导出字段：

| 列名 | 说明 |
|---|---|
| 时间戳 | 原始时间戳 HH:MM:SS.mmm |
| 绝对时间 | 同时间戳（冗余列，方便筛选排序） |
| 相对时间(s) | 距首帧的秒数，保留 3 位小数 |
| 功能码 | 如 0x82 |
| 参数 | 如 0x00 |
| 校验通过 | True/False |
| *(各数据字段)* | 如 Roll, Pitch, Yaw 等 |

## 遇到的问题与解决方案

### 1. 图例点击切换通道无效

**现象**：点击图例条目无法隐藏/显示对应曲线。

**原因**：matplotlib 的 `pick_event` 在 Qt5Agg/QtAgg 后端下完全不触发（经诊断脚本验证触发次数为 0）。

**解决**：改用 `button_press_event`，通过 `legend.get_texts()` 获取各条目文本的屏幕坐标位置，手动判断点击落在哪个条目上。

### 2. VSCode 中交互功能全部失效

**现象**：从 VSCode 终端运行时，无法缩放、无法悬停显示数值、无法点击图例。

**原因**：VSCode Python 扩展注入 `MPLBACKEND=inline` 环境变量，导致 matplotlib 使用非交互式后端；且 `plt.show()` 不阻塞，事件循环无法启动。

**解决**：
- 启动时清除 `MPLBACKEND` 环境变量并强制选择 `QtAgg` 后端
- 使用纯 `PureQtWindow` 类包装 `FigureCanvasQTAgg`，完全绕过 `plt.show()`
- 通过 `QApplication.exec_()` 直接管理 Qt 事件循环

### 3. 缩放/平移后出现光标残影

**现象**：缩放或平移操作后，屏幕上偶发残留旧位置的垂直虚线或圆点标记。

**原因**：blitting 机制中 `_setup_blit()` 在 `draw_idle()` 尚未执行时就捕获背景，此时旧光标元素仍渲染在画布上，被"烘焙"进背景图中。后续 `restore_region` 恢复的背景本身就包含旧光标，形成残影。

**解决**：在平移/缩放开始时立即调用 `_hide_cursor()` 隐藏所有光标元素并标记 `_blit_invalid = True`，确保下次 `_setup_blit()` 执行 `draw()` 时画布上无光标元素，捕获的背景是干净的。

### 4. 右键拖拽平移卡顿

**现象**：右键拖拽平移坐标系时明显卡顿，不够流畅。

**原因**：每次 `motion_notify_event`（鼠标每移动一个像素）都调用 `transData.inverted().transform()` 做坐标逆变换，该操作涉及完整的仿射变换矩阵运算，开销较大。

**解决**：
- 在 `_on_click`（平移开始时）预计算像素→数据的缩放因子 `pan_sx` / `pan_sy`，平移时简化为一次乘法
- 降低 min-max 抽取目标（3000 → 2000 点），减少每帧渲染顶点数
- 提高路径简化阈值（`path.simplify_threshold` 0.5 → 0.8），进一步减少 Agg 渲染开销

### 5. 信息框中文显示为方块

**现象**：左上角信息框中"时间"等中文字符显示为方块（tofu）。

**原因**：`info_text` 的字体列表首位为 `Consolas`，不支持 CJK 字符。

**解决**：字体列表首位改为 `Microsoft YaHei`（微软雅黑），确保中文字符正确渲染。

### 6. 程序刚启动时出现残影

**现象**：程序启动后，窗口中偶发残留光标元素的影子。移动或缩放坐标系后残影消失。

**原因**：Windows 11 下 Qt 默认会在绘制前清除窗口背景（`WA_OpaquePaintEvent` 未设置），导致 canvas 首帧渲染时出现闪烁/残影。同时 blit 背景在鼠标首次移入时捕获，此时 canvas 可能尚未完成渲染。

**解决**：
- 设置 `canvas.setAttribute(Qt.WA_OpaquePaintEvent, True)` 禁止 Qt 清除背景
- `show()` 后调用 `fig.canvas.draw()` + `QApplication.processEvents()` 强制同步渲染并刷新 Qt 绘制管线

### 7. Y 轴平移方向反转

**现象**：右键拖拽平移时，向上拖拽坐标系反而向下移动（Y 轴方向相反）。

**原因**：预计算缩放因子优化时，`ddy` 的计算多加了一次负号：`ddy = -(event.y - pan_start_y) * pan_sy`。而公式 `new_ylim = pan_ylim - ddy` 中的减法已经处理了屏幕坐标与数据坐标的方向转换，双重取反导致 Y 轴方向反转。

**解决**：去掉 `ddy` 计算中的负号，改为 `ddy = (event.y - pan_start_y) * pan_sy`。屏幕 Y 向下为正，乘以正的比例因子后，`pan_ylim - ddy` 自然实现正确的平移方向。

### 8. 平移仍然卡顿（深度优化）

**现象**：预计算缩放因子后平移仍有明显卡顿。

**原因**：`draw_idle()` 每次都要完整重绘所有曲线（5 条线 × 2000 点 = 10000 顶点），加上网格、图例等元素，每帧渲染开销约 30-50ms，无法达到 60fps。

**解决**：
- 平移开始时临时将 `path.simplify_threshold` 提高到 1.0（最大简化，跳过更多共线顶点），释放后恢复原值
- 降低 min-max 抽取目标（2000 → 1500 点），进一步减少渲染顶点数
- 配合预计算缩放因子，每帧渲染开销降至 15-25ms

### 9. 0x82 无温度帧解析失败（温度可选）

**现象**：固件把 0x82 帧里的温度字节注释掉后，导出的 CSV 只有基础列（时间戳、功能码等），没有 Roll/Pitch/Yaw/MagDisturb，波形也无法显示。

**原因**：`parse_cmd_fields` 对 0x82 强制要求 `len(data) >= 12`（含温度的帧 data=12 字节）。固件注释温度后帧变短（data 仅 10 字节），所有 0x82 帧被判定非法并丢弃，导致角度数据全部丢失。

**解决**：将温度改为可选项——`len(data) >= 10` 即解析 Roll、Pitch、Yaw、MagDisturb；仅当 `len(data) >= 12` 时才额外解析 `Temp`（data[10]/data[11]）。同一份日志中两种帧（含/不含温度）可混用，缺失的 Temp 自动以 NaN 留空，不影响其他曲线和导出。

## 文件说明

| 文件 | 说明 |
|---|---|
| `visualize_6axis.py` | 主程序（协议解析 + 交互式可视化 + 数据导出） |
| `程序架构分析.md` | 详细的程序架构文档（含伪代码、Bug 修复记录） |
| `README.md` | 本文件 |
