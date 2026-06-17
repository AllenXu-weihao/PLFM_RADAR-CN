# AERIS-10：开源脉冲线性调频（PLFM）相控阵雷达系统

[![GitHub stars](https://img.shields.io/github/stars/NawfalMotii79/PLFM_RADAR?style=social)](https://github.com/NawfalMotii79/PLFM_RADAR/stargazers)
[![Hardware: CERN-OHL-P](https://img.shields.io/badge/Hardware-CERN--OHL--P-blue.svg)](https://ohwr.org/cern_ohl_p_v2.txt)
[![Software: MIT](https://img.shields.io/badge/Software-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Status: Alpha](https://img.shields.io/badge/Status-Alpha-orange)](https://github.com/NawfalMotii79/PLFM_RADAR)
[![Frequency: 10.5GHz](https://img.shields.io/badge/Frequency-10.5GHz-blue)](https://github.com/NawfalMotii79/PLFM_RADAR)

> **原项目作者**：Nawfal Motii (ABAC INDUSTRY, 摩洛哥) | **GitHub ⭐ 21.7k+**
>
> 本文档为 **AERIS-10 相控阵雷达系统的中文版完整说明文档**，基于原始项目整理翻译。

---

## 一、项目概述

### 1.1 什么是 AERIS-10？

**AERIS-10** 是一个**开源、低成本、X 波段（10.5 GHz）相控阵雷达系统**，采用 **脉冲线性调频（Pulse LFM / PLFM）** 调制技术。项目提供完整的硬件设计（原理图、PCB 布局、Gerber 文件）、固件代码（STM32 + FPGA）和上位机软件（Python GUI），面向研究人员、无人机开发者和高阶 SDR 爱好者。

![AERIS-10 天线阵列](8_Utils/Antenna_Array.jpg)

### 1.2 核心特性

| 特性 | 说明 |
|------|------|
| 📡 **工作频率** | X 波段 10.5 GHz |
| 🔓 **完全开源** | 硬件（CERN-OHL-P）+ 软件（MIT）双许可 |
| 🎯 **双版本配置** | Nexus 3km 短距 / Extended 20km 远距 |
| 📐 **电子波束扫描** | 方位角 + 俯仰角 ±45° 电子扫描 |
| ⚡ **FPGA 信号处理** | 脉冲压缩、多普勒 FFT、MTI、CFAR 全链路 |
| 🖥️ **Python GUI** | 实时目标显示 + 地图集成 |
| 🛰️ **GPS/IMU 集成** | 实时位置和姿态校正 |
| 🔧 **模块化设计** | 电源管理、频率合成、射频前端独立分板 |

---

## 二、系统版本对比

AERIS-10 提供两种配置版本，满足不同应用场景需求：

| 参数 | **AERIS-10N（Nexus 近程版）** | **AERIS-10E（Extended 远程版）** |
|------|------|------|
| **最大探测距离** | ~3 km | ~20 km |
| **天线阵列** | 8×16 贴片天线阵 | 32×16 介质填充开槽波导阵 |
| **输出功率** | ~1W × 16 通道 | 10W × 16 通道（GaN 功放） |
| **功率放大器板** | 不需要 | 16 块 QPA2962 GaN 功放板 |
| **适用场景** | 实验室教学、近距离探测 | 远距离监控、无人机防御 |
| **成本** | 较低 | 较高 |

### 关键差异说明

- **天线增益**：远程版采用介质填充开槽波导天线，增益显著高于贴片天线
- **发射功率**：远程版每通道使用 QPA2962 GaN 功率放大器（10W），近程版使用 ADTR1107 内部功放（~1W）
- **接收灵敏度**：两者共享相同的接收链路架构

---

## 三、硬件系统架构

### 3.1 系统总体结构图

![AERIS-10 系统架构图](8_Utils/RADAR_V6_V2.png)

### 3.2 各子系统详解

#### ① 电源管理板（Power Management Board）

- 为所有电子元器件提供所需电压等级
- 具备完善的滤波和**上电/断电时序控制**（由 MCU 保证）
- 电源管理方案详见 `3_Power Management/Power Management V6.xlsx`

#### ② 频率综合器板（Frequency Synthesizer Board）

核心芯片：**AD9523-1** 低抖动时钟发生器

为以下模块提供相位对齐的时钟参考：
- RX / TX 频率综合器（ADF4382）
- DAC（AD9708）
- ADC（AD9484）
- FPGA（XC7A50T / XC7A200T）

#### ③ 主板（Main Board）—— 系统核心

主板集成了雷达系统的绝大多数关键组件：

| 组件 | 型号/说明 | 功能 |
|------|-----------|------|
| **DAC** | AD9708 | 生成雷达 PLFM 调制波形（Chirp） |
| **微波混频器 ×2** | LTC5552 | 上变频（TX链路）+ 中频下变频（RX链路） |
| **4 通道移相器 ×4** | ADAR1000 | RX/TX 链路波束赋形（共 16 通道） |
| **前端芯片 ×16** | ADTR1107 | RX：低噪声放大（LNA）；TX：功率放大（PA） |
| **FPGA** | XC7A50T-2FTG256I | 雷达信号全数字处理（见下文详细说明） |
| **MCU** | STM32F746xx | 系统管理与外设控制（见下文详细说明） |
| **ADC** | AD9484 | 500 MSPS 高速模数转换 |
| **I²C ADC ×2** | ADS7830（U88@0x48 / U89@0x4A） | 16 路 Idq 测量（每 PA 通道一路） |
| **I²C DAC ×2** | DAC5578（U7@0x48 / U69@0x49） | 16 路 Vg 控制（PA 栅极电压闭环校准） |
| **I²C ADC ×1** | ADS7830（U10@0x4B） | 8 路温度传感器读取（热监控） |
| **RF 开关** | M3SWA2-34DR+ | TX/RX 切换与保护 |

##### FPGA 功能模块（XC7A50T / XC7A200T）

FPGA 是整个系统的信号处理核心，负责：

```
┌─────────────────────────────────────────────────────┐
│                  FPGA 信号处理流水线                   │
├─────────────────────────────────────────────────────┤
│ 1. ADC 数据接口        ← AD9484 500MSPS 原始数据      │
│ 2. I/Q 正交下变频      ← NCO 400MHz 数字混频          │
│ 3. CIC 抽取滤波        ← 4 倍抽取 + 增强 CIC          │
│ 4. FIR 低通滤波        ← 多相 FIR 滤波                │
│ 5. 匹配滤波/脉冲压缩   ← PLFM Chirp 相关处理           │
│    ├── 短距离模式：单段匹配滤波                         │
│    └── 长距离模式：多段拼接匹配滤波                     │
│ 6. 距离单元抽取         ← 降低数据率                    │
│ 7. FFT 引擎            ← 1024 点或 2048 点 FFT        │
│ 8. 多普勒处理器        ← 多普勒 FFT + MTI 对消        │
│ 9. CFAR 恒虚警检测     ← 单元平均 CFAR               │
│ 10. 混合 AGC 控制       ← FPGA/STM32/GUI 跨层闭环     │
│ 11. USB 数据接口       ← FT2232H / FT601             │
└─────────────────────────────────────────────────────┘
```

**关键 Verilog 模块文件** (`9_Firmware/9_2_FPGA/`)：

| 模块文件 | 功能 |
|----------|------|
| `radar_system_top.v` | 系统顶层（单一事实来源） |
| `radar_receiver_final.v` | 接收链路顶层集成 |
| `radar_transmitter.v` | 发射链路（DAC Chirp 生成） |
| `plfm_chirp_controller.v` | PLFM 波形控制器 |
| `ddc_400m.v` | 数字下变频（400 MHz） |
| `nco_400m_enhanced.v` | 增强型 NCO（数控振荡器） |
| `cic_decimator_4x_enhanced.v` | 4×增强型 CIC 抽取器 |
| `fir_lowpass.v` | FIR 低通滤波器 |
| `matched_filter_processing_chain.v` | 匹配滤波处理链 |
| `matched_filter_multi_segment.v` | 多段匹配滤波（长距离模式） |
| `frequency_matched_filter.v` | 频域匹配滤波 |
| `fft_engine.v` | FFT 处理引擎 |
| `doppler_processor.v` | 多普勒处理器 |
| `mti_canceller.v` | MTI 动目标显示对消器 |
| `cfar_ca.v` | CFAR 单元平均恒虚警检测 |
| `rx_gain_control.v` | 接收增益控制（AGC 部分） |
| `usb_data_interface.v` / `usb_data_interface_ft2232h.v` | USB 数据接口 |
| `radar_mode_controller.v` | 雷达工作模式控制器 |
| `ad9484_interface_400m.v` | AD9484 高速 ADC 接口 |
| `dac_interface_single.v` | DAC 接口 |
| `cdc_modules.v` | 跨时钟域（CDC）处理 |

##### STM32 微控制器功能（STM32F746xx）

MCU 是系统的"大脑"，负责协调所有外设：

```
┌──────────────────────────────────────────────────┐
│            STM32F746xx 功能概览                   │
├──────────────────────────────────────────────────┤
│ ▸ 电源上电/断电时序控制                            │
│ ▸ 与 FPGA 通信                                   │
│ ▸ AD9523-1 时钟发生器配置                         │
│ ▸ ADF4382 频率综合器配置（×2）                     │
│ ▸ ADAR1000 移相器配置（×4，脉冲时序控制）          │
│ ▸ PA 栅极电压 Vg 校准（DAC5578，启动闭环标定）     │
│ ▸ PA 静态电流 Idq 监测（ADS7830 + INA241A）       │
│ ▸ GPS 定位（UM982，GUI 地图中心 + 目标打标签）     │
│ ▸ IMU 姿态感知（GY-85，俯仰/横滚坐标修正）         │
│ ▸ 气压高度计（BMP180）                             │
│ ▸ 步进电机驱动（机械方位扫描）                      │
│ ▸ 温度监控与散热风扇控制                           │
│ ▸ USB 通信协议处理                                │
└──────────────────────────────────────────────────┘
```

#### ④ 功率放大器板（Power Amplifier Board）— 仅 Extended 版本

- 采用 **QPA2962 GaN HEMT** 功率放大器
- 单通道输出功率：**10W**
- 共 16 块功放板（对应 16 个天线通道）
- 具备过温保护和电流监控

#### ⑤ 天线阵列（Antenna Arrays）

| 版本 | 类型 | 规模 | 特点 |
|------|------|------|------|
| **Nexus (10N)** | 微带贴片天线 | 8×16 = 128 单元 | 成本低、易加工 |
| **Extended (10E)** | 介质填充开槽波导 | 32×16 = 512 单元 | 高增益、低损耗 |

天线仿真工具：
- **openEMS**（开源电磁仿真）：`5_Simulations/Antenna/`
- **MATLAB** 方向图计算：`5_Simulations/Matlab/Antenna_array.m`
- KiCad 设计文件：`4_Schematics and Boards Layout/4_6_Schematics/Antennas/`

#### ⑥ 其他机械部件

- **滑环（Slip-Ring）**：支持 360° 连续旋转的电力/信号传输
- **步进电机 + 驱动器**：机械方位扫描
- **散热风扇 + 散热器**：主动热管理
- **外壳/机箱（Enclosure）**：机械图纸在 `8_Utils/Mechanical_Drawings/`

---

## 四、信号处理流水线详解

### 4.1 发射链路（TX Chain）

```
FPGA(DAC) → LPF → LTC5552(上变频) → ADAR1000(移相) → ADTR1107(PA) → 天线
   ↓            ↓           ↓              ↓            ↓
 PLFM Chirp   重构滤波    10.5GHz RF     波束赋形     功率放大
```

**PLFM（脉冲线性调频）调制特点**：
- 发射短时宽带宽的 Chirp（线性调频）脉冲
- 通过匹配滤波实现**脉冲压缩**——将能量集中到窄脉冲，同时获得距离分辨率
- 相比传统 CW/FMCW 雷达：峰值功率高、作用距离远、抗干扰能力强

### 4.2 接收链路（RX Chain）

```
天线 → ADTR1107(LNA) → ADAR1000(移相) → LTC5552(下变频) → AD9484(ADC) → FPGA
  ↓          ↓              ↓              ↓              ↓           ↓
 回波信号   低噪声放大     波束合成        中频(IF)      数字化       全处理
```

### 4.3 FPGA 数字信号处理流程

```
回波信号 → ADC采样 → I/Q下变频 → CIC抽取 → FIR滤波 → 匹配滤波(脉压)
                                                    ↓
                        ←←←←←← 距离维处理 ←←←←←←←←←
                                                    ↓
                          FFT(距离-多普勒地图 RDM) → 多普勒处理
                                                    ↓
                                              MTI(动目标对消)
                                                    ↓
                                            CFAR(恒虚警检测)
                                                    ↓
                                              USB → GUI 显示
```

**各阶段说明**：

| 处理阶段 | 算法 | 作用 |
|----------|------|------|
| **I/Q 下变频** | NCO 数字正交混频 | 将 IF 信号搬移到基带 |
| **CIC 抽取** | 级联积分梳状滤波器 | 降低采样率，节省后续计算资源 |
| **FIR 低通** | 多相 FIR | 抗混叠 + 通道整形 |
| **脉冲压缩** | 匹配滤波（相关运算） | 提高距离分辨率 + 信噪比 |
| **FFT** | 快速傅里叶变换 | 生成距离像 |
| **多普勒 FFT** | 慢时间维度 FFT | 提取速度信息 |
| **MTI** | 动目标显示对消 | 抑制静止杂波 |
| **CFAR** | 单元平均恒虚警 | 自适应阈值检测目标 |

---

## 五、软件系统

### 5.1 Python GUI 上位机

位置：`9_Firmware/9_3_GUI/`

提供多个版本的图形界面：

| 版本 | 文件 | UI 框架 | 说明 |
|------|------|---------|------|
| V6 | `GUI_V6.py` | Tkinter | 基础版本（已弃用） |
| V65 | `GUI_V65_Tk.py` | Tkinter | 当前稳定版（推荐） |
| V7 | `GUI_V7_PyQt.py` | PyQt6 | 新一代界面（开发中） |

**功能包括**：
- 实时 PPI（平面位置指示器）显示
- 距离-多普勒地图（RDM）可视化
- AGC（自动增益控制）分析仪表盘
- 地图集成（基于 GPS 定位）
- 雷达参数实时调整

### 5.2 固件代码结构

```
9_Firmware/
├── 9_1_Microcontroller/          # STM32 固件
│   ├── 9_1_1_C_Cpp_Libraries/    # 外设驱动库（SPI/I²C/UART 等）
│   ├── 9_1_2_C_Cpp_Algorithms/   # 信号处理算法文档
│   ├── 9_1_3_C_Cpp_Code/         # 主程序 main.cpp
│   └── tests/                    # MCU 单元测试（cpputest）
├── 9_2_FPGA/                     # FPGA RTL
│   ├── *.v                       # Verilog 源码
│   ├── *.mem                     # 存储器初始化文件（Chirp LUT 等）
│   ├── constraints/              # XDC 约束文件
│   ├── scripts/                  # Vivado 构建脚本
│   ├── formal/                   # 形式验证（SymbiYosys）
│   └── tb/                       # 测试平台（iverilog + xSim）
├── 9_3_GUI/                      # Python 上位机
│   ├── v7/                       # V7 PyQt6 模块化包
│   └── requirements*.txt         # 依赖声明
├── tests/cross_layer/            # 跨层契约测试（FPGA-MCU-GUI）
└── tools/                        # 工具脚本（UART 抓包等）
```

---

## 六、仿真与验证

### 6.1 射频/电磁仿真（`5_Simulations/`）

| 仿真类别 | 工具 | 内容 |
|----------|------|------|
| **天线仿真** | openEMS / MATLAB | 介质填充开槽波导、贴片天线方向图 |
| **AAV（孔径天线）** | openEMS | 孔径天线仿真 |
| **DAC 重构滤波器** | QucsStudio | DAC 输出滤波器设计 |
| **IF 带通滤波器** | QucsStudio | 平衡/非平衡 BPF 设计 |
| **枝节 BPF** | QucsStudio | TE 模枝节滤波器 |
| **波导** | Sonnet EM | 氧化铝基板波导仿真 |
| **过孔围栏** | openEMS / QucsStudio | 过孔隔离效果 |
| **QPA2962 功放** | QucsStudio | GaN 功放电路仿真 |
| **RF 开关** | QucsStudio + S 参数 | 开关阻抗分析 |
| **阵列方向图** | MATLAB | Kaiser 窗加权方向图计算 |

### 6.2 FPGA 验证（`9_Firmware/9_2_FPGA/tb/`）

- **单元测试**：每个 Verilog 模块都有独立 testbench
- **回归测试**：`run_regression.sh` 自动运行 5 个阶段（Lint → 模块 → 集成 → 信号处理 → P0 对抗测试）
- **形式验证**：`formal/` 目录包含 SymbiYosys 形式化属性检查
- **跨层测试**：`tests/cross_layer/` 验证 FPGA-MCU-GUI 协议一致性

---

## 七、PCB 制造与装配

### 7.1 生产文件（`4_Schematics and Boards Layout/4_7_Production Files/`）

| 文件夹 | 内容 | PCB 层数 |
|--------|------|----------|
| `Gerber_Main_Board` | 主板 Gerber + BOM/CPL | 10 层 |
| `Gerber_freq_synth` | 频率综合器板 | 6 层 |
| `Gerber_PA` | 功率放大器板 | 4 层 |
| `Gerber_Patch_Antenna` | 贴片天线板 | 4 层 |
| `Gerber_PowerBoard` | 电源管理板 | 2 层 |

**板材**：Rogers RO4350B（高频板），阻抗控制参考文件已包含。

### 7.2 EDA 工具

- **原理图 / PCB**：Eagle（`.sch` / `.brd` 格式）
- **FPGA 综合**：Xilinx Vivado（XC7A50T / XC7A200T）
- **开发板兼容**：TE0712 / TE0713（FMC 接口）、UMFT601X（HSDIO）

---

## 八、开源许可协议

本项目采用 **硬件/软件分离许可**：

### 硬件部分 — CERN-OHL-P v2

适用于：
- 原理图与 PCB 布局（`4_Schematics and Boards Layout/`）
- BOM 表、Gerber 文件
- 机械图纸与外壳设计

**权利**：可使用、修改、销售基于这些设计的产品  
**义务**：保留版权通知、修改需以源码格式发布、修改后仍以相同许可分发

### 软件/固件部分 — MIT License

适用于：
- FPGA 代码（Verilog/VHDL，`9_Firmware/9_2_FPGA/`）
- STM32 固件（C/C++，`9_Firmware/9_1_Microcontroller/`）
- Python GUI 与工具（`9_Firmware/9_3_GUI/`）
- 所有仿真脚本与分析工具

完整许可文本参见仓库根目录 `Licence` 文件。

---

## 九、快速开始

### 9.1 前置条件

- 基础雷达原理知识
- PCB 焊接组装经验（如需自制硬件）
- **Python 3.12+**（运行 GUI）
- **Xilinx Vivado**（修改 FPGA 逻辑时需要）

### 9.2 硬件装配步骤

```bash
# 1. 订购 PCB
# 生产文件位于: 4_Schematics and Boards Layout/4_7_Production Files/

# 2. 采购元器件
# BOM/CPL 文件与 Gerber 同目录

# 3. 参考原理图焊接
# 原理图位于: 4_Schematics and Boards Layout/4_6_Schematics/

# 4. 选择对应版本的天线
# Nexus:  Antennas/Patch/
# Extended: Antennas/Waveguide/

# 5. 机械加工外壳
# 图纸位于: 8_Utils/Mechanical_Drawings/
```

### 9.3 运行软件

```bash
# 安装 GUI 依赖（V65 Tkinter 版本）
cd 9_Firmware/9_3_GUI
pip install -r requirements_dashboard.txt

# 启动雷达 GUI
python GUI_V65_Tk.py

# 或运行 V7 PyQt6 版本
pip install -r requirements_v7.txt
python GUI_V7_PyQt.py
```

### 9.4 运行测试

```bash
# 代码风格检查
uv run ruff check .

# Python 测试
cd 9_Firmware/9_3_GUI && uv run pytest test_GUI_V65_Tk.py test_v7.py -v

# FPGA 回归测试
cd 9_Firmware/9_2_FPGA && bash run_regression.sh

# MCU 单元测试
cd 9_Firmware/9_1_Microcontroller/tests && make clean && make

# 跨层契约测试
uv run pytest 9_Firmware/tests/cross_layer/test_cross_layer_contract.py -v
```

---

## 十、仓库目录结构总览

```
PLFM_RADAR/
├── 1_Project_Description/                 # 项目描述文档
├── 2_Functional Diagram & Interconnection Matrices/  # 功能框图 & 连接矩阵
├── 3_Power Management/                    # 电源管理方案（Excel）
├── 4_Schematics and Boards Layout/        # 原理图 + PCB 布局 + Gerber + BOM
│   ├── 4_4_Board Stack-up/                # 板材堆叠说明
│   ├── 4_6_Schematics/                    # Eagle 原理图（5 类板卡）
│   └── 4_7_Production Files/              # 生产文件（Gerber/BOM/CPL）
├── 5_Simulations/                         # 射频/电磁/算法仿真
├── 6_Application Notes/                   # 应用笔记（PDF）
├── 7_Components Datasheets and Application notes/  # 元器件数据手册
├── 8_Utils/                               # 图片、CAD 库、机械图、Python 工具
├── 9_Firmware/                            # ★ 核心固件代码
│   ├── 9_1_Microcontroller/               # STM32 固件（C/C++）
│   │   ├── 9_1_1_C_Cpp_Libraries/         # 外设驱动库
│   │   ├── 9_1_2_C_Cpp_Algorithms/        # 算法文档
│   │   ├── 9_1_3_C_Cpp_Code/              # 主程序
│   │   └── tests/                         # 单元测试
│   ├── 9_2_FPGA/                          # FPGA 信号处理（Verilog）
│   │   ├── *.v / *.mem                    # RTL + 存储器初始化
│   │   ├── constraints/                   # XDC 约束
│   │   ├── scripts/                       # 构建脚本（200t/50t/te0712/te0713）
│   │   ├── formal/                        # 形式验证
│   │   └── tb/                            # 测试平台 + 黄金参考
│   ├── 9_3_GUI/                           # Python 上位机（Tkinter/PyQt6）
│   │   └── v7/                            # V7 模块化包
│   ├── tests/cross_layer/                 # 跨层契约测试
│   └── tools/                             # UART 抓包等工具
├── docs/                                  # GitHub Pages 文档站
│   ├── artifacts/                         # 已发布的 Bitstream
│   └── assets/img/                        # 文档图片资源
├── .github/workflows/ci-tests.yml         # CI/CD 配置
├── README.md                              # 英文原始 README
├── README_CN.md                           # ★ 中文说明文档（本文档）
├── CONTRIBUTING.md                        # 贡献指南
├── Licence                                # CERN-OHL-P 许可证全文
├── pyproject.toml                         # Python 项目配置（Ruff linting）
└── .gitignore                             # Git 忽略规则
```

---

## 十一、致谢

> *"这个项目始于摩洛哥的一个小工作室。今天，19,000 名工程师在 GitHub 上为它点亮了星星。"*
> —— Nawfal Motii, ABAC INDUSTRY

**原作者**：Nawfal Motii ([GitHub](https://github.com/NawfalMotii79))  
**机构**：ABAC INDUSTRY（[www.abacindustry.com](http://www.abacindustry.com)）  
**赞助商**：PCBWay（PCB 打样制造）

**中文版整理**：基于原始开源项目整理翻译，旨在帮助中文社区更好地理解和使用这一优秀的开源雷达项目。

---

⭐ 如果你对开源雷达技术感兴趣，欢迎 Star 这个项目！

*注：本项目处于活跃开发中，部分功能仍在完善中。请查看 Issues 页面了解已知限制和即将发布的特性。*
