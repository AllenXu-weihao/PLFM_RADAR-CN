# AERIS-10 相控阵雷达 FPGA 架构说明

> 本文档对 `PLFM_RADAR` 项目的 FPGA 固件进行全面解析，涵盖系统架构、信号链路、各模块功能及关键设计细节。

---

## 目录

1. [系统概述](#1-系统概述)
2. [顶层架构](#2-顶层架构)
3. [信号处理流水线](#3-信号处理流水线)
4. [核心模块详解](#4-核心模块详解)
5. [时钟域与跨时钟处理](#5-时钟域与跨时钟处理)
6. [DSP48E1 优化技术](#6-dsp48e1-优化技术)
7. [主机命令集](#7-主机命令集)
8. [关键参数表](#8-关键参数表)

---

## 1. 系统概述

AERIS-10 是一款开源 FMCW（调频连续波）相控阵雷达系统，FPGA 固件实现完整的信号收发链路：

- **调制方式**：PLFM（分段线性调频）/ LFM
- **扫描方式**：电子扫描 + 机械扫描
- **ADC 采样率**：400 MSPS（AD9484）
- **DAC 更新率**：120 MSPS（AD9122）
- **主要目标**：实现近距离高精度测距（短啁啾）+ 远距离探测（长啁啾）

### FPGA 平台

| 项目 | 内容 |
|------|------|
| 主要目标板 | Xilinx Artix-7 XC7A200T |
| 开发环境 | Vivado |
| 工程创建 | Tcl 脚本（`build_200t.tcl`） |
| 源代码语言 | Verilog HDL |

---

## 2. 顶层架构

### 2.1 系统顶层（`radar_system_top.v`）

系统顶层模块集成以下主要子系统：

```
                ┌─────────────────────────────────────────────────┐
                │           radar_system_top                      │
                │                                                 │
  100MHz ───────┤  clk_100m         ┌─────────────────────┐    │
  120MHz ───────┤  clk_120m_dac     │  radar_transmitter  │    │
  FT601_CLK ────┤  ft601_clk        │  (发射机顶层)        │    │
                │                    └──────────┬──────────┘    │
                │                               │ DAC_data      │
                │                    ┌──────────┴──────────┐    │
                │                    │  radar_receiver_final │    │
                │                    │  (接收机顶层)         │    │
                │                    └──────────┬──────────┘    │
                │                               │ detect_valid  │
                │                    ┌──────────┴──────────┐    │
                │                    │  cfar_ca            │    │
                │                    │  (CFAR检测器)        │    │
                │                    └──────────┬──────────┘    │
                │                               │               │
                │                    ┌──────────┴──────────┐    │
                │                    │  usb_data_interface  │    │
                │                    │  (USB数据传输)        │    │
                │                    └─────────────────────┘    │
                └─────────────────────────────────────────────────┘
```

### 2.2 主要接口

| 接口 | 方向 | 描述 |
|------|------|------|
| `clk_100m` | 输入 | 100MHz 系统时钟 |
| `clk_120m_dac` | 输入 | 120MHz DAC 时钟 |
| `ft601_clk` | 输入 | USB FT601 时钟 |
| `adc_data` | 输入 | 8位 ADC 数据（400MHz DDR） |
| `dac_data` | 输出 | 14位 DAC 数据 |
| `spi_*_1v8` | 输出 | SPI 电平转换后信号（1.8V） |
| `ft601_*` | 双向 | FT601 USB3.0 接口 |
| `led_*` | 输出 | 状态 LED |

### 2.3 USB 模式选择

通过 `USB_MODE` 参数选择 USB 接口类型：

- `USB_MODE = 0`：FT601（32位 USB3.0）
- `USB_MODE = 1`：FT2232H（8位 USB2.0）

使用 `generate` 块根据参数选择实例化不同的 USB 接口模块。

---

## 3. 信号处理流水线

### 3.1 完整信号链路

```
ADC (400MHz LVDS)
  │
  ▼
ad9484_interface_400m        ← IDDR 双沿采样，400MSPS
  │
  ▼
ddc_400m_enhanced            ← NCO混频 + CIC 4x抽取 + FIR低通
  │  (400MHz → 100MHz CDC)
  ▼
ddc_input_interface           ← 跨时钟域数据缓冲
  │
  ▼
rx_gain_control              ← 数字增益控制 / AGC
  │
  ▼
matched_filter_multi_segment  ← 脉冲压缩（匹配滤波）
  │
  ▼
range_bin_decimator           ← 距离维抽取（1024 → 64 bin）
  │
  ▼
mti_canceller                ← 杂波抑制（可选3脉冲对消）
  │
  ▼
doppler_processor            ← 双16点FFT（速度处理）
  │
  ▼
cfar_ca                      ← CFAR自适应检测
  │
  ▼
usb_data_interface            ← USB传输至上位机
```

### 3.2 数据速率变化

| 阶段 | 采样率 | 数据宽度 | 说明 |
|------|---------|----------|------|
| ADC 输入 | 400 MHz | 8 bit | LVDS DDR |
| DDC 后 | 100 MHz | 18 bit | CIC 4x抽取 + FIR |
| 匹配滤波后 | 100 MHz | 18 bit | 脉冲压缩输出 |
| 距离抽取后 | 100 MHz | 18 bit | 1024 → 64 bin |
| Doppler 后 | 100 MHz | 10 bit | 5bit sub_frame + 5bit bin |
| CFAR 后 | 事件驱动 | 检测报告 | 仅输出超过门限的目标 |

---

## 4. 核心模块详解

### 4.1 接收机顶层 — `radar_receiver_final.v`

**文件**：`9_Firmware/9_2_FPGA/radar_receiver_final.v`（501 行）

**功能**：实例化完整的接收信号处理链。

**关键参数**：

| 参数 | 值 | 说明 |
|------|-----|------|
| `INPUT_BINS` | 1024 | 输入距离 bin 数量 |
| `OUTPUT_BINS` | 64 | 输出距离 bin 数量 |
| `DECIMATION_FACTOR` | 16 | 距离维抽取因子 |

**接口信号**：

```verilog
// 时钟与复位
input  wire        clk_400m,        // 400MHz ADC 时钟
input  wire        clk_100m,        // 100MHz 系统时钟
input  wire        rst_n,            // 异步复位（低有效）

// ADC 接口
input  wire [7:0] adc_data,         // ADC 并行数据

// 控制接口
input  wire        chirp_active,     // 啁啾激活信号
input  wire        mti_enable,       // MTI 使能
input  wire [2:0] range_bin_select, // 距离 bin 选择

// 输出
output wire [9:0] final_result,     // 最终结果（含 Doppler 信息）
output wire        final_valid       // 结果有效信号
```

---

### 4.2 ADC 接口 — `ad9484_interface_400m.v`

**文件**：`9_Firmware/9_2_FPGA/ad9484_interface_400m.v`（169 行）

**功能**：通过 IDDR 原语捕获 ADC LVDS 双沿数据，实现 400MSPS 采样。

**时钟方案**：

```
ADC_CLK_P/N → IBUFDS → BUFIO (零延迟，驱动 IDDR)
                      → BUFG  (抖动清理，驱动 FPGA fabric)
                      → MMCM (抖动清理)
```

**IDDR 模式**：`SAME_EDGE_PIPELINED`，输出在单时钟沿稳定，便于后续处理。

**关键代码段**：

```verilog
// IDDR 原语实例化（捕获双沿数据）
IDDR #(
    .DDR_CLK_EDGE("SAME_EDGE_PIPELINED"),
    .INIT_Q1(1'b0),
    .INIT_Q2(1'b0),
    .SRTYPE("SYNC")
) iddr_inst (
    .Q1(adc_q1),   // 上升沿数据
    .Q2(adc_q2),   // 下降沿数据
    .C(clk_400m_bufio),
    .CE(1'b1),
    .D(adc_data_p),
    .R(~rst_n),
    .S(1'b0)
);
```

---

### 4.3 数字下变频 — `ddc_400m.v`

**文件**：`9_Firmware/9_2_FPGA/ddc_400m.v`（787 行）

**功能**：在 400MHz 时钟域实现完整的数字下变频。

#### 4.3.1 架构框图

```
                 ┌──────────────────────────────────────┐
                 │           ddc_400m_enhanced          │
                 │                                      │
  adc_data ──────┤  ADC符号转换                        │
  (8bit)         │      │                               │
                 │      ▼                               │
                 │  NCO (6级流水)                      │
                 │  sin/cos 查找表                     │
                 │      │                               │
                 │      ▼                               │
                 │  混频器 (DSP48E1 × 2)              │
                 │  I = adc × cos                       │
                 │  Q = adc × (-sin)                   │
                 │      │                               │
                 │      ▼                               │
                 │  CIC 5级 4x抽取                     │
                 │  (DSP48E1 级联)                     │
                 │      │                               │
                 │      ▼  100MHz                       │
                 │  FIR 32抽头低通                      │
                 │      │                               │
                 │      ▼                               │
                 │  CDC (Gray-code同步)                │
                 │      │                               │
                 │      ▼  100MHz 系统时钟域            │
                 │  ddc_i/q (18bit) ───────────────────┤
                 └──────────────────────────────────────┘
```

#### 4.3.2 NCO 设计

NCO 采用 6 级流水线设计，使用 DSP48E1 做相位累加器：

| 级 | 功能 | 关键操作 |
|----|------|----------|
| Stage 1 | 相位累加 | DSP48E1 累加模式（P = P + C） |
| Stage 2 | 偏移加 | 加上频率控制字 |
| Stage 3a | 象限判断 + LUT 索引 | 取相位的 [15:10] 位 |
| Stage 3b | LUT 读取 | 64点 1/4 波正弦表 |
| Stage 4 | 取反 | 根据象限决定是否取反 |
| Stage 5 | 符号应用 | 输出最终 sin/cos 值 |

**LUT 实现**：使用 `ram_style = "distributed"` 强制使用 LUTRAM（而非 BRAM），满足 400MHz 时序要求。

#### 4.3.3 混频器优化

混频器使用两个 DSP48E1 实例（I 和 Q 通道），全流水线设计：

```verilog
// DSP48E1 配置
AREG = 1,   // A 输入寄存器
BREG = 1,   // B 输入寄存器
MREG = 1,   // M 累加寄存器
PREG = 1    // P 输出寄存器
```

这样实现了 4 级流水，关键路径被打散，满足 400MHz 时序。

#### 4.3.4 复位同步器

400MHz 下复位去断言存在时序违例风险，使用 2 级异步断言+同步释放复位同步器：

```verilog
// 复位同步器（抑制 400MHz 复位去断言时序违例）
reg [1:0] rst_sync_400m;
always @(posedge clk_400m or negedge rst_n) begin
    if (!rst_n)
        rst_sync_400m <= 2'b00;
    else
        rst_sync_400m <= {rst_sync_400m[0], 1'b1};
end
wire rst_n_400m = rst_sync_400m[1];  // 同步释放的复位信号
```

---

### 4.4 CIC 抽取滤波器 — `cic_decimator_4x_enhanced.v`

**文件**：`9_Firmware/9_2_FPGA/cic_decimator_4x_enhanced.v`（903 行）

**功能**：5 级 CIC 滤波器，实现 4x 抽取。

#### 4.4.1 CIC 原理

CIC（Cascaded Integrator-Comb）滤波器结构：

```
积分器（Integrator）× 5 ── 抽取 4x ── 梳状器（Comb）× 5
```

- **积分器**：在高速时钟域（400MHz）工作
- **梳状器**：在低速时钟域（100MHz）工作
- **优势**：无需乘法器，适合高速应用

#### 4.4.2 DSP48E1 级联优化

使用 DSP48E1 的 PCOUT→PCIN 专用级联布线，将 5 级积分器级联：

```verilog
// 第1级 DSP48E1（积分器）
DSP48E1 #(
    .AREG(1),
    .BREG(1),
    .MREG(1),
    .PREG(1),
    .USE_PCIN("FALSE"),
    .USE_PCOUT("TRUE")   // 使能 PCOUT 输出
) dsp_integrator_0 (...);

// 第2级 DSP48E1（积分器）
DSP48E1 #(
    .USE_PCIN("TRUE"),   // 使用 PCIN 输入（来自上一级 PCOUT）
    .USE_PCOUT("TRUE")
) dsp_integrator_1 (...);
```

这样避免了 fabric 布线延迟，保证 400MHz 时序收敛。

#### 4.4.3 Comb 部分优化

Comb 部分使用 DSP48E1 的 CREG=1 吸收组合逻辑关键路径：

```verilog
// Comb 0 使用 CREG=1 消除 0.643ns 布线延迟
DSP48E1 #(
    .CREG(1),    // C 输入寄存器使能（关键！）
    .AREG(0),
    .BREG(0),
    .MREG(0),
    .PREG(1)
) dsp_comb_0 (...);
```

---

### 4.5 FIR 低通滤波器 — `fir_lowpass.v`

**文件**：`9_Firmware/9_2_FPGA/fir_lowpass.v`（318 行）

**功能**：32 抽头低通 FIR 滤波器，9 级流水。

#### 4.5.1 滤波器参数

| 参数 | 值 |
|------|-----|
| 抽头数 | 32 |
| 数据宽度 | 18 bit |
| 系数宽度 | 18 bit |
| 累加宽度 | 36 bit |
| 流水线级数 | 9 |

#### 4.5.2 加法树结构

使用 5 级加法树将 32 个乘积缩减为 1 个：

```
32 乘积 → 16 和 → 8 和 → 4 和 → 2 和 → 1 最终和
 │         │       │       │       │
 └─────────┴───────┴───────┴───────┘
  每级使用 USE_DSP="no" 强制在 fabric 中执行
  （节省 DSP48 给 FFT 使用）
```

#### 4.5.3 饱和逻辑（已修复）

原始代码存在饱和逻辑阈值 bug，已修复为正确的 18 位有符号范围：

```verilog
// 修复后的饱和逻辑
if (accumulator_reg > $signed({{(ACCUM_WIDTH-DATA_WIDTH){1'b0}}, {(DATA_WIDTH-1){1'b1}}})) begin
    data_out <= (2**(DATA_WIDTH-1))-1;   // +131071 (最大值)
end else if (accumulator_reg < $signed({1'b1, {(ACCUM_WIDTH-1){1'b0}}})) begin
    data_out <= -(2**(DATA_WIDTH-1));     // -131072 (最小值)
end
```

---

### 4.6 匹配滤波器 — `matched_filter_multi_segment.v`

**文件**：`9_Firmware/9_2_FPGA/matched_filter_multi_segment.v`（541 行）

**功能**：使用重叠保留法实现脉冲压缩（匹配滤波）。

#### 4.6.1 重叠保留法

将 3000 样本长啁啾分段为 4 个 1024 点 segment：

```
┌─────────────────────────────────────────┐
│           3000 样本啁啾                 │
├────────┬────────┬────────┬────────────┤
│ Seg 0  │ Seg 1  │ Seg 2  │ Seg 3    │
│ 1024点 │ 1024点 │ 1024点 │ 1024点    │
└────────┴────────┴────────┴────────────┘
     │        │        │        │
     ▼        ▼        ▼        ▼
  零填充到 1024 点
     │
     ▼
  FFT (使用 xfft_1024 IP)
     │
     ▼
  频域相乘（参考啁啾 × 接收啁啾）
     │
     ▼
  IFFT
     │
     ▼
  去除前 128 点（重叠部分）
  保留 896 点（SEGMENT_ADVANCE）
```

#### 4.6.2 状态机

```verilog
localparam [2:0]
    ST_IDLE           = 3'b000,
    ST_COLLECT_DATA   = 3'b001,
    ST_ZERO_PAD       = 3'b010,
    ST_WAIT_REF       = 3'b011,
    ST_PROCESSING     = 3'b100,
    ST_WAIT_FFT       = 3'b101,
    ST_OUTPUT         = 3'b110,
    ST_NEXT_SEGMENT   = 3'b111,
    ST_OVERLAP_COPY   = 3'b???;  // 实际代码中为 3'bXXX
```

**关键参数**：

| 参数 | 值 | 说明 |
|------|-----|------|
| `SEGMENT_ADVANCE` | 896 | 每个 segment 前进样本数 |
| `OVERLAP_SAMPLES` | 128 | 重叠样本数 |
| `NUM_SEGMENTS` | 4 | segment 数量 |

---

### 4.7 增益控制 — `rx_gain_control.v`

**文件**：`9_Firmware/9_2_FPGA/rx_gain_control.v`（283 行）

**功能**：数字增益控制 + 可选 AGC，位于 DDC 和匹配滤波器之间。

#### 4.7.1 增益编码

增益值 `gain_shift[3:0]` 的编码方式：

| 位 | 功能 |
|----|------|
| `gain_shift[3]` | 方向（0 = 放大，1 = 衰减） |
| `gain_shift[2:0]` | 量（0-7，表示移位位数） |

例如：
- `4'b0000` = 增益 1x（无变化）
- `4'b0001` = 增益 2x（左移 1 位）
- `4'b1001` = 增益 1/2x（右移 1 位）

#### 4.7.2 AGC 算法

AGC（自动增益控制）根据每帧的 `saturation_count` 和 `peak_magnitude` 调整增益：

```
if (saturation_count > threshold_high)
    gain_shift = gain_shift + attack_step;   // 快速衰减
else if (peak_magnitude < threshold_low)
    gain_shift = gain_shift - decay_step;    // 缓慢放大
else
    holdoff_counter++;                       // 保持
```

---

### 4.8 MTI 杂波抑制 — `mti_canceller.v`

**文件**：`9_Firmware/9_2_FPGA/mti_canceller.v`（216 行）

**功能**：对地杂波抑制，支持 2 脉冲和 3 脉冲对消。

#### 4.8.1 对消原理

| 模式 | 传递函数 | 频率响应 |
|------|----------|----------|
| 2 脉冲 | H(z) = 1 - z⁻¹ | 高通（抑制零频） |
| 3 脉冲 | H(z) = 1 - 2z⁻¹ + z⁻² | 更高阶高通 |

#### 4.8.2 3 脉冲对消实现

使用 `prev` 和 `prev2` 双缓冲区实现二阶对消：

```verilog
// 3 脉冲对消：H(z) = 1 - 2z⁻¹ + z⁻²
wire signed [DATA_WIDTH+1:0] diff3_i_full =
    {diff_i_full[DATA_WIDTH], diff_i_full} + prev2_minus_prev_i;

// prev2_minus_prev_i = prev_i - prev2_i (已在上一时钟周期计算)
```

---

### 4.9 Doppler 处理器 — `doppler_processor.v`

**文件**：`9_Firmware/9_2_FPGA/doppler_processor.v`（536 行）

**功能**：Staggered-PRF Doppler 处理器，解决速度模糊问题。

#### 4.9.1 Staggered PRF 原理

使用两个不同的 PRI（脉冲重复间隔）：

- **长 PRI**：高精度测速，但存在速度模糊
- **短 PRI**：速度模糊解算

通过中国余数定理解算真实速度。

#### 4.9.2 双 16 点 FFT

对长 PRI 和短 PRI 分别做 16 点 FFT：

```
帧缓冲（2048×16bit BRAM）
  │
  ▼
汉明窗加权
  │
  ▼
xfft_16 IP（长 PRI）
  │
  ▼
xfft_16 IP（短 PRI）
  │
  ▼
输出格式：{sub_frame, bin[3:0]}
  bit[4]   = sub_frame (0=长PRI, 1=短PRI)
  bit[3:0] = Doppler bin 索引
```

---

### 4.10 CFAR 检测器 — `cfar_ca.v`

**文件**：`9_Firmware/9_2_FPGA/cfar_ca.v`（561 行）

**功能**：CA/GO/SO-CFAR 自适应阈值检测器。

#### 4.10.1 CFAR 原理

CFAR（Constant False Alarm Rate）通过在检测单元两侧设置保护单元和参考窗，估计局部噪声功率，自适应设置检测门限。

```
参考窗    保护单元    检测单元    保护单元    参考窗
─────── ──────── ───────── ──────── ────────
███████   ████     ☆        ████    ███████
        ← 左半窗 → <-保护-> <-- 右半窗 ->
```

#### 4.10.2 三种 CFAR 模式

| 模式 | 编码 | 描述 |
|------|------|------|
| CA-CFAR | `2'b00` | 平均前后窗噪声功率 |
| GO-CFAR | `2'b01` | 选择平均功率大的一侧（多目标场景） |
| SO-CFAR | `2'b10` | 选择平均功率小的一侧（边带干扰场景） |

#### 4.10.3 幅值近似优化

使用 `max + min/2` 近似计算幅值，节省平方根运算：

```verilog
wire [15:0] abs_i = data_i[15] ? (-data_i) : data_i;
wire [15:0] abs_q = data_q[15] ? (-data_q) : data_q;

wire [15:0] mag_max = (abs_i > abs_q) ? abs_i : abs_q;
wire [15:0] mag_min = (abs_i > abs_q) ? abs_q : abs_i;
wire [MAG_WIDTH-1:0] cur_mag = {1'b0, mag_max} + {2'b0, mag_min[15:1]}; // max + min/2
```

误差约 5%，但节省大量逻辑资源。

---

### 4.11 发射机顶层 — `radar_transmitter.v`

**文件**：`9_Firmware/9_2_FPGA/radar_transmitter.v`（249 行）

**功能**：包含 SPI 电平转换、边沿检测、CDC、PLFM 啁啾控制器和 DAC 接口。

**关键设计**：

1. **SPI 电平转换**：3.3V ↔ 1.8V（使用外部电平转换芯片，FPGA 内部做同步）
2. **边沿检测**：所有 STM32 GPIO 输入都经过 `cdc_single_bit` 同步器消除亚稳态
3. **PLFM 啁啾控制器**：生成线性调频信号的控制字
4. **DAC 接口**：驱动 AD9122 DAC

---

### 4.12 模式控制器 — `radar_mode_controller.v`

**文件**：`9_Firmware/9_2_FPGA/radar_mode_controller.v`（394 行）

**功能**：控制波束扫描和啁啾模式。

#### 4.12.1 工作模式

| 模式 | 描述 |
|------|------|
| STM32 控制模式 | 由 STM32 通过 SPI 命令控制所有参数 |
| 自动扫描模式 | FPGA 自动完成波束扫描和啁啾序列 |
| 单啁啾调试模式 | 用于调试，单次啁啾 |

#### 4.12.2 状态机

```
S_IDLE ────→ S_LONG_CHIRP ──→ S_LONG_LISTEN
                                      │
                                      ▼
                               S_GUARD ──→ S_SHORT_CHIRP ──→ S_SHORT_LISTEN
                                                                        │
                                                                        ▼
                                                                 S_ADVANCE ──→ S_IDLE
```

**时序参数**（100MHz 周期）：

| 状态 | 计数 | 时间 |
|------|------|------|
| `LONG_CHIRP` | 3000 | 30 μs |
| `LONG_LISTEN` | 13700 | 137 μs |
| `GUARD` | 17540 | 175.4 μs |
| `SHORT_CHIRP` | 50 | 0.5 μs |
| `SHORT_LISTEN` | 17450 | 174.5 μs |

---

### 4.13 CDC 模块库 — `cdc_modules.v`

**文件**：`9_Firmware/9_2_FPGA/cdc_modules.v`（271 行）

**功能**：提供多种跨时钟域处理方案。

#### 4.13.1 可用 CDC 模块

| 模块名 | 功能 | 适用场景 |
|--------|------|----------|
| `cdc_single_bit` | 单比特同步器（多级触发器） | 复位、使能等单比特信号 |
| `cdc_gray_code` | Gray-code 多比特 CDC | 指针、计数器等多比特信号 |
| `cdc_toggle` | Toggle 同步器 | 脉冲信号跨时钟 |
| `cdc_handshake` | 带握手的 CDC | 数据有效信号跨时钟 |
| `cdc_adc_to_processing` | ADC→处理链 CDC | 专门为多通道数据设计的 CDC |

#### 4.13.2 Toggle CDC 原理

```verilog
// 发送域：每次脉冲翻转 toggle 信号
always @(posedge clk_src or negedge rst_n) begin
    if (!rst_n)
        toggle_src <= 1'b0;
    else if (pulse_src)
        toggle_src <= ~toggle_src;
end

// 接收域：同步 toggle 信号，边沿检测产生接收域脉冲
always @(posedge clk_dst or negedge rst_n) begin
    if (!rst_n)
        {toggle_dst2, toggle_dst1} <= 2'b00;
    else
        {toggle_dst2, toggle_dst1} <= {toggle_dst1, toggle_src_sync};
end
assign pulse_dst = toggle_dst2 ^ toggle_dst1;  // 边沿检测
```

---

## 5. 时钟域与跨时钟处理

### 5.1 时钟域汇总

| 时钟域 | 频率 | 来源 | 用途 |
|--------|------|------|------|
| `clk_400m` | 400 MHz | ADC Clock | ADC 接口、DDC、CIC |
| `clk_120m_dac` | 120 MHz | DAC Clock | 发射机、NCO |
| `clk_100m` | 100 MHz | 系统时钟 | 接收机处理后级、USB 接口 |
| `ft601_clk` | 可变 | USB Clock | USB 数据传输 |

### 5.2 跨时钟路径

| 从 | 到 | 方法 |
|----|-----|------|
| `clk_400m` (ADC) | `clk_100m` (系统) | Gray-code + Toggle 同步 |
| `clk_120m_dac` (DAC) | `clk_100m` (系统) | 多级触发器同步 |
| STM32 GPIO (异步) | `clk_100m` (系统) | `cdc_single_bit` 同步器 |
| `clk_100m` (系统) | `ft601_clk` (USB) | 异步 FIFO |

---

## 6. DSP48E1 优化技术

本项目大量使用 DSP48E1 原语级优化，以下是一些关键技术：

### 6.1 PCOUT→PCIN 级联

用于 CIC 滤波器的 5 级积分器级联，避免 fabric 布线延迟。

### 6.2 流水线策略

- **AREG/BREG**：输入寄存器，吸收输入路径延迟
- **MREG**：乘法器输出寄存器，打散关键路径
- **PREG**：输出寄存器，保证时序收敛

### 6.3 显式实例化 vs 推断

本项目选择**显式实例化** DSP48E1 原语，而非依赖综合器推断，原因：

1. 精确控制流水线级数
2. 保证 400MHz 时序收敛
3. 可直接使用 PCOUT→PCIN 级联
4. 避免综合器在不同时刻做出不同推断

### 6.4 USE_DSP 属性

在 `fir_lowpass.v` 中，加法树使用 `USE_DSP="no"` 强制在 fabric 中执行，节省 DSP48 给 FFT 使用。

---

## 7. 主机命令集

主机（STM32/PC）通过 SPI/USB 发送命令，命令格式为 1 字节命令码 + N 字节参数。

### 7.1 雷达配置命令

| 命令码 | 名称 | 参数 | 描述 |
|--------|------|------|------|
| `0x01` | SET_RADAR_MODE | 1 | 设置雷达工作模式 |
| `0x02` | SET_CHIRP_PARAM | 4 | 设置啁啾参数 |
| `0x03` | SET_CFR_CONFIG | 2 | 设置 CFAR 配置 |
| `0x04` | SET_GAIN | 1 | 设置增益 |
| `0x05` | SET_MTI_ENABLE | 1 | 使能/禁止 MTI |
| `0x06` | SET_AGC_ENABLE | 1 | 使能/禁止 AGC |
| `0x07` | SET_SCAN_MODE | 1 | 设置扫描模式 |
| `0x08` | SET_BEAM_ANGLE | 2 | 设置波束角度 |

### 7.2 自检与校准命令

| 命令码 | 名称 | 描述 |
|--------|------|------|
| `0x20` | START_SELF_TEST | 启动自检 |
| `0x21` | CALIBRATE_DC_OFFSET | 校准 DC 偏移 |
| `0x22` | CALIBRATE_IQ_IMBALANCE | 校准 IQ 不平衡 |

---

## 8. 关键参数表

### 8.1 系统参数

| 参数 | 值 | 说明 |
|------|-----|------|
| ADC 采样率 | 400 MSPS | AD9484 |
| DAC 更新率 | 120 MSPS | AD9122 |
| 系统时钟 | 100 MHz | |
| 啁啾长度 | 3000 样本（长）/ 50 样本（短） | |
| 匹配滤波器分段数 | 4 | 重叠保留法 |
| FFT 点数（Doppler） | 16 | 双 FFT（长/短 PRI） |
| CFAR 参考窗长度 | 16 | 前后各 8 |
| CFAR 保护单元数 | 2 | 前后各 1 |

### 8.2 资源估算（XC7A200T）

| 资源类型 | 用量估算 | 利用率 |
|----------|----------|--------|
| LUT | ~15,000 | ~11% |
| FF | ~20,000 | ~7% |
| DSP48E1 | ~48 | ~21% |
| BRAM | ~20 | ~14% |

---

## 附录 A：文件清单

| 文件 | 行数 | 功能 |
|------|------|------|
| `radar_system_top.v` | 1078 | 系统顶层 |
| `radar_receiver_final.v` | 501 | 接收机顶层 |
| `radar_transmitter.v` | 249 | 发射机顶层 |
| `radar_mode_controller.v` | 394 | 模式控制器 |
| `ddc_400m.v` | 787 | 数字下变频 |
| `cic_decimator_4x_enhanced.v` | 903 | CIC 抽取滤波器 |
| `nco_400m_enhanced.v` | 368 | NCO 数字振荡器 |
| `fir_lowpass.v` | 318 | FIR 低通滤波器 |
| `matched_filter_multi_segment.v` | 541 | 匹配滤波器 |
| `rx_gain_control.v` | 283 | 增益控制/AGC |
| `mti_canceller.v` | 216 | MTI 杂波抑制 |
| `doppler_processor.v` | 536 | Doppler 处理器 |
| `cfar_ca.v` | 561 | CFAR 检测器 |
| `ad9484_interface_400m.v` | 169 | ADC LVDS 接口 |
| `cdc_modules.v` | 271 | CDC 模块库 |
| `usb_data_interface.v` | - | USB 数据传输（FT601） |
| `usb_data_interface_ft2232h.v` | - | USB 数据传输（FT2232H） |

---

## 附录 B：参考资料

1. **FMCW 雷达原理**：https://en.wikipedia.org/wiki/Frequency-modulated_continuous-wave_radar
2. **CIC 滤波器**：Hogenauer, E. (1981). An economical class of digital filters for decimation and interpolation.
3. **CFAR 检测**：Rohling, H. (1983). Radar CFAR thresholding in clutter and multiple target situations.
4. **Xilinx DSP48E1**：UG479 7 Series DSP48E1 Slice User Guide
5. **Vivado 设计流程**：UG892 Vivado Design Suite User Guide

---

*文档版本：v1.0*  
*生成日期：2026-06-18*  
*作者：Nova 🌑（基于源码分析）*
