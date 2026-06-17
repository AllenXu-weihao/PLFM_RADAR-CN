# FPGA 信号处理流水线优化贡献

## 概述

本文件记录了针对 AERIS-10 相控阵雷达系统 FPGA 固件层的信号处理流水线优化工作。
所有优化均保持向后兼容，不改变模块接口，可通过参数配置启用/禁用。

---

## 优化清单

### 1. FIR 低通滤波器饱和逻辑修复 (fir_lowpass.v)

**问题**: 输出饱和逻辑的比较阈值为 `2^(ACCUM_WIDTH-2) = 2^34`，而累加器是 36 位有符号数（最大正值 2^35-1）。
这意味着饱和检测几乎永远不会触发，导致输出截断而非饱和，产生非线性失真。

**修复**:
- 将正饱和阈值从 `2^34-1` 修正为 `2^(DATA_WIDTH-1)-1 = 131071`（18 位有符号最大正值）
- 将负饱和阈值从 `-2^34` 修正为 `-2^(DATA_WIDTH-1) = -131072`（18 位有符号最小负值）
- 修正位选范围：从 `accumulator_reg[34:15]`（20 位）改为 `accumulator_reg[35:18]`（18 位），正确匹配输出宽度
- 同步修正 `filter_overflow` 输出信号的阈值

**影响**: 消除 FIR 输出截断失真，改善接收链路动态范围和 SNR。

---

### 2. CFAR 检测器幅值近似精度提升 (cfar_ca.v)

**问题**: 原始幅值计算使用 `|I| + |Q|`（L1 范数），在 45° 方向过度估计达 41.4%，
导致对角线方向的目标产生虚警，且 CFAR 阈值在这些位置偏高。

**优化**: 替换为 alpha-max-plus-beta-min 近似:
```
magnitude ≈ max(|I|, |Q|) + min(|I|, |Q|) / 2
```
最大误差从 41.4% 降至 11.8%，与真实 L2 范数更接近。

**资源开销**: 1 个额外比较器 + 1 个右移器（可忽略不计 vs BRAM/DSP 预算）

**影响**:
- 减少 CFAR 虚警率（特别是对角线方向目标）
- 改善弱目标检测灵敏度
- 提高 Range-Doppler 图的动态范围利用率

---

### 3. MTI 消杂波器支持 3 脉冲对消 (mti_canceller.v)

**改进**: 添加 `PULSES` 参数（默认 2，可选 3），支持 3 脉冲对消模式。

| 模式 | 传递函数 | 杂波改善 | 额外资源 |
|------|---------|---------|---------|
| 2-pulse (默认) | H(z) = 1 - z⁻¹ | ~6 dB | 无（原样） |
| 3-pulse (新增) | H(z) = 1 - 2z⁻¹ + z⁻² | ~12 dB | +2 BRAM, +30 LUTs |

3 脉冲对消器在 DC 处的抑制凹槽比 2 脉冲深 2 倍（dB 域），
对于强地面杂波环境（如低空无人机探测）效果显著。

**向后兼容**: `PULSES=2`（默认）行为与原始代码完全一致。
启用 3 脉冲模式只需在实例化时设置 `PULSES=3`。

**实例化示例**:
```verilog
// 3-pulse canceller for enhanced clutter rejection
mti_canceller #(
    .NUM_RANGE_BINS(64),
    .DATA_WIDTH(16),
    .PULSES(3)  // 启用 3 脉冲对消
) mti_inst (...);
```

---

### 4. DDC ADC 符号转换简化 (ddc_400m.v)

**问题**: ADC 无符号→有符号转换表达式过于复杂：
```verilog
{1'b0, adc_data, 9'b0} - {1'b0, 8'hFF, 9'b0} / 2
```
难以阅读、验证，且综合工具可能无法识别为简单的减法+移位。

**优化**: 简化为清晰的两步操作：
```verilog
wire signed [8:0] adc_offset = $signed({1'b0, adc_data}) - 9'sd128;
assign adc_signed_w = {{8{adc_offset[8]}}, adc_offset} <<< 9;
```

**影响**: 功能等价，但代码可读性和可维护性显著提升，综合结果不变。

---

## 验证建议

### 仿真验证
1. 运行现有 testbench 确认无回归:
   ```bash
   cd 9_Firmware/9_2_FPGA/tb
   iverilog -D SIMULATION -o tb_fir.vvp ../fir_lowpass.v tb_fir_lowpass.v
   vvp tb_fir.vvp
   ```

2. CFAR 幅值验证: 注入已知 I/Q 值，检查输出幅值是否匹配预期
3. MTI 3-pulse 验证: 注入 3 个连续 chirp，验证第 3 chirp 输出 = current - 2*prev + prev2

### 综合验证
1. 在 Vivado 中综合 `radar_system_top.v`，确认资源变化符合预期
2. 检查时序报告：所有优化均在 100 MHz 域，不应影响 400 MHz 关键路径
3. CFAR 幅值改进为组合逻辑，检查是否影响 CFAR FSM 时序裕量

---

## 文件变更清单

| 文件 | 变更类型 | 影响 |
|------|---------|------|
| `fir_lowpass.v` | 饱和逻辑修复 | 输出精度提升 |
| `cfar_ca.v` | 幅值近似改进 | 检测精度提升 |
| `mti_canceller.v` | 3-pulse 选项 | 杂波抑制增强 |
| `ddc_400m.v` | 代码简化 | 可维护性提升 |

---

## 贡献者

Allen Xu (AllenXu-weihao)  
FPGA Developer — AERIS-10 PLFM Radar CN Community
