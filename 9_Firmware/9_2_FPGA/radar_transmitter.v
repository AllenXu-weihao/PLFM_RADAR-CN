`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: AERIS-10 相控阵雷达项目
// Engineer: FPGA Team
// 
// Create Date:    19:04:35 12/14/2025 
// Design Name:    雷达发射机顶层模块
// Module Name:    radar_transmitter 
//
// 【中文功能概述】
// 本模块是 AERIS-10 雷达系统的发射机顶层，负责：
// 1. SPI 电平转换（STM32F7 的 3.3V ↔ ADAR1000 波束形成器的 1.8V）
// 2. STM32 GPIO 输入的边沿检测和 CDC 同步
// 3. PLFM 啁啾信号生成（线性调频波形）
// 4. DAC 接口（驱动 AD9122 输出模拟中频信号）
// 5. RF 开关和混频器使能控制
// 6. ADAR1000 波束形成器控制信号输出
//
// 【时钟域】
// - clk_100m (100MHz)：系统时钟，用于边沿检测和 CDC
// - clk_120m_dac (120MHz)：DAC 时钟，用于啁啾生成和 DAC 驱动
//
// 【关键设计】
// - 所有异步输入都经过 CDC 同步器消除亚稳态
// - new_chirp 脉冲使用 Toggle CDC 跨时钟域传输（避免脉冲丢失）
// - 混频器使能信号同步到 120MHz 域后控制发射链路
//////////////////////////////////////////////////////////////////////////////////

module radar_transmitter(
    // ========== 系统时钟（System Clocks） ==========
    input wire clk_100m,           // 系统主时钟 100MHz
    input wire clk_120m_dac,       // DAC 时钟 120MHz（驱动 AD9122）
    input wire reset_n,            // 复位（已同步到 clk_120m_dac 域，低有效）
    input wire reset_100m_n,       // 复位（已同步到 clk_100m 域，低有效）
    
    // ========== DAC 接口（DAC Interface） ==========
    output wire [7:0] dac_data,    // DAC 数据输出（8位，至 AD9122）
    output wire dac_clk,           // DAC 时钟输出
    output wire dac_sleep,         // DAC 低功耗控制（1=休眠）
    output wire rx_mixer_en,       // 接收混频器使能
    output wire tx_mixer_en,       // 发射混频器使能
	 
    // ========== STM32 控制接口（STM32 Control Interface） ==========
    input wire stm32_new_chirp,        // 新啁啾触发脉冲（来自 STM32 GPIO）
    input wire stm32_new_elevation,    // 新俯仰角触发脉冲
    input wire stm32_new_azimuth,      // 新方位角触发脉冲
    input wire stm32_mixers_enable,    // 混频器总使能（高有效）
	 
    output wire fpga_rf_switch,        // RF 收发开关控制（0=接收, 1=发送）
	 
    // ========== ADAR1000 波束形成器接口 ==========
    // 每个 ADAR1000 有 TX/RX 加载信号和 TR（收发切换）信号
    output wire adar_tx_load_1,   // ADAR#1 TX 寄存器加载
    output wire adar_rx_load_1,   // ADAR#1 RX 寄存器加载
    output wire adar_tx_load_2,   // ADAR#2 TX 寄存器加载
    output wire adar_rx_load_2,   // ADAR#2 RX 寄存器加载
    output wire adar_tx_load_3,   // ADAR#3 TX 寄存器加载
    output wire adar_rx_load_3,   // ADAR#3 RX 寄存器加载
    output wire adar_tx_load_4,   // ADAR#4 TX 寄存器加载
    output wire adar_rx_load_4,   // ADAR#4 RX 寄存器加载
    output wire adar_tr_1,        // ADAR#1 收发切换
    output wire adar_tr_2,        // ADAR#2 收发切换
    output wire adar_tr_3,        // ADAR#3 收发切换
    output wire adar_tr_4,        // ADAR#4 收发切换
    
    // ========== SPI 电平转换接口（Level Shifter SPI Interface）==========
    // FPGA 桥接 3.3V STM32 总线（Bank 15）到 1.8V ADAR1000 总线（Bank 34）
    // I/O Bank 本身处理电压转换；此处 assign 只是布线通过 fabric
    input wire stm32_sclk_3v3,         // STM32 SPI 时钟（3.3V 域）
    input wire stm32_mosi_3v3,         // STM32 SPI MOSI（3.3V 域）
    output wire stm32_miso_3v3,        // STM32 SPI MISO（3.3V 域）
    input wire stm32_cs_adar1_3v3,     // ADAR#1 片选（3.3V 域）
    input wire stm32_cs_adar2_3v3,     // ADAR#2 片选（3.3V 域）
    input wire stm32_cs_adar3_3v3,     // ADAR#3 片选（3.3V 域）
    input wire stm32_cs_adar4_3v3,     // ADAR#4 片选（3.3V 域）
    
    output wire stm32_sclk_1v8,        // SPI 时钟（1.8V 域，至 ADAR1000）
    output wire stm32_mosi_1v8,        // SPI MOSI（1.8V 域）
    input wire stm32_miso_1v8,         // SPI MISO（1.8V 域，来自 ADAR1000）
    output wire stm32_cs_adar1_1v8,    // ADAR#1 片选（1.8V 域）
    output wire stm32_cs_adar2_1v8,    // ADAR#2 片选（1.8V 域）
    output wire stm32_cs_adar3_1v8,    // ADAR#3 片选（1.8V 域）
    output wire stm32_cs_adar4_1v8,    // ADAR#4 片选（1.8V 域）
	 
    // ========== 波束位置跟踪输出（Beam Position Tracking） ==========
    output wire [5:0] current_elevation,   // 当前俯仰角波束编号（0~30）
    output wire [5:0] current_azimuth,     // 当前方位角波束编号（0~49）
    output wire [5:0] current_chirp,       // 当前啁啾编号（0~31）
    output wire new_chirp_frame            // 新啁啾帧标志（每个新啁啾置位）
);
	 
// ========== SPI 电平转换直通 ==========
// FPGA 桥接 3.3V STM32 SPI 总线到 1.8V ADAR1000 SPI 总线。
// FPGA I/O Bank 本身处理实际电压转换（Bank 15 = 3.3V，Bank 34 = 1.8V），
// 这些 assign 只是让信号通过 fabric 布线。
assign stm32_sclk_1v8      = stm32_sclk_3v3;     // SPI 时钟：3.3V → 1.8V
assign stm32_mosi_1v8       = stm32_mosi_3v3;      // SPI MOSI：3.3V → 1.8V
assign stm32_miso_3v3       = stm32_miso_1v8;       // SPI MISO：1.8V → 3.3V（回读方向）
assign stm32_cs_adar1_1v8   = stm32_cs_adar1_3v3;   // ADAR#1 片选
assign stm32_cs_adar2_1v8   = stm32_cs_adar2_3v3;   // ADAR#2 片选
assign stm32_cs_adar3_1v8   = stm32_cs_adar3_3v3;   // ADAR#3 片选
assign stm32_cs_adar4_1v8   = stm32_cs_adar4_3v3;   // ADAR#4 片选

// ========== 边沿检测信号（Edge Detection Signals） ==========
wire new_chirp_pulse;          // 新啁啾脉冲（clk_100m 域）
wire new_elevation_pulse;      // 新俯仰角脉冲
wire new_azimuth_pulse;        // 新方位角脉冲

// ========== CDC 同步信号 ==========
// 异步 STM32 GPIO 输入同步到 clk_100m 域
wire stm32_new_chirp_sync;     // 已同步的新啁啾信号
wire stm32_new_elevation_sync; // 已同步的俯仰角信号
wire stm32_new_azimuth_sync;   // 已同步的方位角信号

// CDC：信号从 clk_100m → clk_120m_dac 域的同步版本
wire mixers_enable_120m;        // 混频器使能（已同步到 120MHz 域）
wire new_chirp_pulse_120m;      // 新啁啾脉冲（Toggle CDC 后在 120MHz 域）

// ========== 啁啾控制信号（Chirp Control Signals） ==========
wire [7:0] chirp_data;         // 啁啾数据（至 DAC）
wire chirp_valid;               // 啁啍数据有效
wire chirp_sequence_done;       // 啁啍序列完成标志

// ========== Toggle CDC：new_chirp 脉冲从 clk_100m → clk_120m_dac ==========
// 边沿检测器在 100MHz 域产生单周期脉冲。由于 120/100 MHz 频率比，
// 电平同步器会丢失这个脉冲。Toggle CDC 将脉冲转换为电平翻转，
// 在目标时钟域同步后再检测边沿来恢复脉冲。
reg chirp_toggle_100m;
always @(posedge clk_100m or negedge reset_100m_n) begin
    if (!reset_100m_n)
        chirp_toggle_100m <= 1'b0;
    else if (new_chirp_pulse)
        chirp_toggle_100m <= ~chirp_toggle_100m;   // 收到脉冲则翻转电平
end

// 将翻转信号同步到 clk_120m_dac 时钟域（3 级触发器消除亚稳态）
wire chirp_toggle_120m;
cdc_single_bit #(.STAGES(3)) cdc_chirp_toggle (   // 3 级同步器
    .src_clk(clk_100m),
    .dst_clk(clk_120m_dac),
    .reset_n(reset_n),
    .src_signal(chirp_toggle_100m),
    .dst_signal(chirp_toggle_120m)
);

// 在 clk_120m_dac 域检测翻转信号边沿，恢复原始脉冲
reg chirp_toggle_120m_prev;
always @(posedge clk_120m_dac or negedge reset_n) begin
    if (!reset_n)
        chirp_toggle_120m_prev <= 1'b0;
    else
        chirp_toggle_120m_prev <= chirp_toggle_120m;
end
assign new_chirp_pulse_120m = chirp_toggle_120m ^ chirp_toggle_120m_prev;  // 异或检测边沿

// 将 stm32_mixers_enable（异步 GPIO 电平）同步到 clk_120m_dac 域
cdc_single_bit #(.STAGES(3)) cdc_mixers_en_120m (
    .src_clk(clk_100m),          // GPIO 是异步的，100MHz 作为伪源时钟
    .dst_clk(clk_120m_dac),
    .reset_n(reset_n),
    .src_signal(stm32_mixers_enable),
    .dst_signal(mixers_enable_120m)
);

// ========== CDC 同步器：异步 STM32 GPIO → clk_100m 域 ==========
// 这些同步器防止边沿检测器出现亚稳态。
// 没有它们，边沿检测器的第一个 FF 可能进入亚稳态，
// XOR 输出可能产生毛刺，导致虚假的啁啾/俯仰/方位脉冲。
cdc_single_bit #(.STAGES(2)) cdc_stm32_chirp (
    .src_clk(clk_100m),         // 异步 GPIO 用 100MHz 作为伪源时钟
    .dst_clk(clk_100m),
    .reset_n(reset_100m_n),
    .src_signal(stm32_new_chirp),
    .dst_signal(stm32_new_chirp_sync)
);

cdc_single_bit #(.STAGES(2)) cdc_stm32_elevation (
    .src_clk(clk_100m),
    .dst_clk(clk_100m),
    .reset_n(reset_100m_n),
    .src_signal(stm32_new_elevation),
    .dst_signal(stm32_new_elevation_sync)
);

cdc_single_bit #(.STAGES(2)) cdc_stm32_azimuth (
    .src_clk(clk_100m),
    .dst_clk(clk_100m),
    .reset_n(reset_100m_n),
    .src_signal(stm32_new_azimuth),
    .dst_signal(stm32_new_azimuth_sync)
);

// ========== 增强型 STM32 输入边沿检测（带去抖动）==========
// 输入已经过 CDC 同步（亚稳态安全）
edge_detector_enhanced chirp_edge (
    .clk(clk_100m),
    .reset_n(reset_100m_n),
    .signal_in(stm32_new_chirp_sync),     // 已同步的啁啾触发
    .rising_falling_edge(new_chirp_pulse)             // 输出：上升/下降沿脉冲
);

edge_detector_enhanced elevation_edge (
    .clk(clk_100m),
    .reset_n(reset_100m_n),
    .signal_in(stm32_new_elevation_sync),  // 已同步的俯仰角触发
    .rising_falling_edge(new_elevation_pulse)
);

edge_detector_enhanced azimuth_edge (
    .clk(clk_100m),
    .reset_n(reset_100m_n),
    .signal_in(stm32_new_azimuth_sync),   // 已同步的方位角触发
    .rising_falling_edge(new_azimuth_pulse)
);

// ========== 增强型 PLFM 啁啾生成器实例化 ==========
// 负责生成线性调频（LFM/PLFM）啁啾波形，控制 ADAR1000 波束形成器，
// 以及管理 RF 开关和混频器时序。
plfm_chirp_controller_enhanced plfm_chirp_inst (
    .clk_120m(clk_120m_dac),
    .clk_100m(clk_100m),
    .reset_n(reset_n),
    .new_chirp(new_chirp_pulse_120m),      // CDC 同步后的脉冲（120MHz 域）
    .new_elevation(new_elevation_pulse),
    .new_azimuth(new_azimuth_pulse),
    .new_chirp_frame(new_chirp_frame),      // 新啁啍帧标志输出
    .mixers_enable(mixers_enable_120m),     // CDC 同步后的混频器使能
    .chirp_data(chirp_data),                // 啁啍数据输出至 DAC
    .chirp_valid(chirp_valid),              // 啁啍数据有效
    .chirp_done(chirp_sequence_done),       // 啁啍序列完成
    .rf_switch_ctrl(fpga_rf_switch),        // RF 开关控制
    .rx_mixer_en(rx_mixer_en),              // 接收混频器使能
    .tx_mixer_en(tx_mixer_en),              // 发射混频器使能
    // ADAR1000 波束形成器控制信号（4 片级联）
    .adar_tx_load_1(adar_tx_load_1),
    .adar_rx_load_1(adar_rx_load_1),
    .adar_tx_load_2(adar_tx_load_2),
    .adar_rx_load_2(adar_rx_load_2),
    .adar_tx_load_3(adar_tx_load_3),
    .adar_rx_load_3(adar_rx_load_3),
    .adar_tx_load_4(adar_tx_load_4),
    .adar_rx_load_4(adar_rx_load_4),
    .adar_tr_1(adar_tr_1),
    .adar_tr_2(adar_tr_2),
    .adar_tr_3(adar_tr_3),
    .adar_tr_4(adar_tr_4),
    // 波束位置计数输出
    .elevation_counter(current_elevation),   // 当前俯仰角（0~30）
    .azimuth_counter(current_azimuth),       // 当前方位角（0~49）
    .chirp_counter(current_chirp)            // 当前啁啍编号（0~31）
);

// ========== 增强 DAC 接口实例化 ==========
// 将啁啍数据转换为 AD9122 所需的时序格式
dac_interface_enhanced dac_interface_inst (
    .clk_120m(clk_120m_dac),
    .reset_n(reset_n),
    .chirp_data(chirp_data),          // 来自 PLFM 控制器的啁啍数据
    .chirp_valid(chirp_valid),         // 啁啍数据有效标志
    .dac_data(dac_data),               // 至 AD9122 的数据
    .dac_clk(dac_clk),                 // 至 AD9122 的时钟
    .dac_sleep(dac_sleep)              // 低功耗控制
);
endmodule
