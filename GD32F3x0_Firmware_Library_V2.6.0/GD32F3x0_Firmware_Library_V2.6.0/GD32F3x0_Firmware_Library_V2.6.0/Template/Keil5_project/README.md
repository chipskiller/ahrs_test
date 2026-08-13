# GD32F330F8 姿态传感器工程（AHRS + OTA）

> 串口姿态解算（ICM42670 + QMC5883P），支持 Bootloader 跳转 + OTA 远程升级。

## 一、硬件

| 器件 | 型号 | 说明 |
|---|---|---|
| 主控 MCU | **GD32F330F8** | Cortex-M4，64KB Flash，8KB SRAM |
| IMU（六轴） | ICM42670 | 加速度计 + 陀螺仪，I2C 地址 `0x68` |
| 磁力计（三轴） | QMC5883P | 地磁，I2C 地址 `0x2C` |

## 二、下载本工程后需要做的事（重要！）

工程使用 **Keil MDK-ARM**。因为源码引用都是**相对路径**，拷到任何位置都能用，但**必须先安装器件支持包**：

### 1. 安装 Keil 器件支持包（Device Family Pack, DFP）

本工程依赖：**`GigaDevice.GD32F3x0_DFP.3.4.0`**

安装方法（任选其一）：
- **在线**：打开 Keil → 菜单 `Pack Installer` → 搜索 `GD32F3x0` → 找到 `GigaDevice GD32F3x0_DFP` → 点 `Install`
- **离线**：从 GigaDevice 官网下载 `.pack` 文件 → 双击安装（或 Pack Installer → File → Import）

> 未安装会提示找不到器件 `GD32F330F8` 或编译报头文件缺失。

### 2. 打开工程

```
Keil5_project\Project.uvprojx
```
双击即可用 Keil 打开（两个 target 都在同一个工程文件里）。

### 3. 选择编译目标（Target）

工程包含**两个 target**，编译/烧录前要选对：

| Target | 用途 | 链接地址 | 大小 |
|---|---|---|---|
| **APP** | 姿态解算主固件 | `0x08002000` | 27KB |
| **BL** | Bootloader 引导程序 | `0x08000000` | 8KB |

在 Keil 工具栏的 Target 下拉框里切换（或 `Project → Select Target`）。

## 三、烧录

1. 接好调试器（JLink/STLink，SWD 接口）
2. 选对 Target → 点 **Download**（或 `F8`）
3. **第一次烧录必须两个都烧**：先烧 BL，再烧 APP
4. 之后升级可通过串口 OTA（协议：`0xF0` 开始 / `0xF1` 写入 / `0xF2` 结束 / `0xF3` 查询版本）

## 四、串口

- USART0：`PA9`=TX，`PA10`=RX，波特率 **9600**，8N1

## 五、Flash 分区（64KB）

```
0x08000000 - 0x08001FFF  Bootloader 8KB
0x08001C00               升级标志（第七页）
0x08002000 - 0x08008BFF  App 27KB
0x08008C00 - 0x0800F7FF  下载区/OTA 27KB
0x0800F800 - 0x0800FFFF  数据区 2KB（yaw / 零点 / 报警）
```

## 六、目录说明（打包/分享时参考）

```
Template/
├── Keil5_project/          ← Keil 工程（打开这里）
│   ├── Project.uvprojx     ← 工程文件（必需）
│   └── output/             ← 编译产物（自动生成，无需打包）
├── main.c / flash.c / ota_protocol.c / hard_i2c.c ...  ← 你的源码（必需）
└── 可视化工具/             ← 上位机可视化脚本（可选）
../../Firmware/             ← GD32 标准外设库（必需，IncludePath 引用它）
../../Utilities/            ← GD32 评估板公用代码（必需）
```

**分享时必须包含**：`Project.uvprojx` + `Template/` 下所有 `.c/.h` + `Firmware/` + `Utilities/`。
**无需包含**：`output/`、`*.map`、`*.lst`、`Project.uvguix.*`、`Project.uvoptx`、`*.log` 等生成物/个人配置。
