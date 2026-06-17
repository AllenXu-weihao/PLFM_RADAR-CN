/**
 * ad9484_interface_400m.v — AD9484 ADC LVDS 接口（400MSPS 双沿采样）
 *
 * 【中文功能概述】
 * 本模块实现 AD9484 高速 ADC 的 LVDS 差分接口，使用 IDDR 原语捕获双沿数据，
 * 实现 400 MSPS 的等效采样率。是整个接收链路的"最前端"——所有数据从这里进入 FPGA。
 *
 * 【AD9484 规格】
 *   - 8 位分辨率，最高 500 MSPS（本系统运行在 400 MHz DCO）
 *   - LVDS 差分数据输出 + LVDS 差分时钟输出（DCO）
 *   - DDR 模式：时钟上下沿各采样一次数据
 *
 * 【时钟方案（关键设计）】
 *   ┌─────────────────────────────────────────────────────┐
 *   │  adc_dco_p/n (400MHz LVDS)                          │
 *   │    → IBUFDS（差分转单端）                            │
 *   │    ├→ BUFIO（零延迟，驱动 IDDR 采样时钟）            │
 *   │    │   ↑ I/O 时钟域专用缓冲器，不进 BUFG 全局网络     │
 *   │    └→ BUFG（抖动清理后驱动 fabric 逻辑）             │
 *   │        → MMCM 进一步去抖动（可选）                    │
 *   └─────────────────────────────────────────────────────┘
 *
 * 【IDDR 采样模式】SAME_EDGE_PIPELINED
 *   - 数据在时钟双沿采样，但在单边沿稳定输出
 *   - Q1 = 上升沿数据，Q2 = 下降沿数据（均寄存化）
 *
 * 【多目标支持】
 *   通过 XDC 约束（非 RTL 参数）适配不同 FPGA 板卡：
 *   - XC7A200T (FBG484)：Bank 14 VCCO = 2.5V → LVDS_25
 *   - XC7A50T  (FTG256)：Bank 14 VCCO = 3.3V → LVDS_33
 *
 * Clock domains:
 *   - 输出域：clk_400m (400MHz, 来自 BUFIO/BUFG)
 *   - 控制域：sys_clk (100MHz, 仅用于复位等慢速控制)
 */

module ad9484_interface_400m (
    // ========== ADC 物理接口（LVDS 差分对）==========
    input wire [7:0] adc_d_p,          // ADC 数据正端 P（8 bit 差分）
    input wire [7:0] adc_d_n,          // ADC 数据负端 N（8 bit 差分）
    input wire adc_dco_p,              // 数据时钟输出正端 P（400MHz LVDS）
    input wire adc_dco_n,              // 数据时钟输出负端 N
    
    // ========== 系统接口 ==========
    input wire sys_clk,                // 100MHz 系统时钟（仅用于控制信号）
    input wire reset_n,                // 异步复位（低有效）
    
    // ========== 400MHz 域输出 ==========
    output wire [7:0] adc_data_400m,   // ADC 数据（8bit，@400MHz 域）
    output wire adc_data_valid_400m,   // 数据有效标志（@400MHz 域）
    output wire adc_dco_bufg           // 缓冲后的 400MHz DCO 时钟（供下游使用）
);

// ========== LVDS 差分转单端（LVDS to Single-Ended Conversion） ==========
wire [7:0] adc_data;     // 单端数据（8 bit）
wire adc_dco;            // 单端 DCO 时钟

// ========== IBUFDS：每根数据线的差分缓冲器 ==========
// 注意：IOSTANDARD 和 DIFF_TERM 通过 XDC 约束设置，而非 RTL 参数，
//       以支持多种 FPGA 目标板的不同 Bank 电压：
//       - XC7A200T (FBG484)：Bank 14 VCCO = 2.5V → LVDS_25
//       - XC7A50T  (FTG256)：Bank 14 VCCO = 3.3V → LVDS_33
genvar i;
generate
    for (i = 0; i < 8; i = i + 1) begin : data_buffers
        IBUFDS #(
            .DIFF_TERM("FALSE"),    // 由 XDC DIFF_TERM 属性覆盖
            .IOSTANDARD("DEFAULT")  // 由 XDC IOSTANDARD 属性覆盖
        ) ibufds_data (
            .O(adc_data[i]),       // 单端输出
            .I(adc_d_p[i]),        // 正端输入 P
            .IB(adc_d_n[i])        // 负端输入 N
        );
    end
endgenerate

// ========== DCO 时钟差分缓冲器 ==========
IBUFDS #(
    .DIFF_TERM("FALSE"),    // Overridden by XDC DIFF_TERM property
    .IOSTANDARD("DEFAULT")  // Overridden by XDC IOSTANDARD property
) ibufds_dco (
    .O(adc_dco),
    .I(adc_dco_p),
    .IB(adc_dco_n)
);

// ============================================================================
// Clock buffering strategy for source-synchronous ADC interface:
//
// BUFIO: Near-zero insertion delay, can only drive IOB primitives (IDDR).
//        Used for IDDR clocking to match the data path delay through IBUFDS.
//        This eliminates the hold violation caused by BUFG insertion delay.
//
// BUFG:  Global clock buffer for fabric logic (downstream processing).
//        Has ~4 ns insertion delay but that's fine for fabric-to-fabric paths.
// ============================================================================
wire adc_dco_bufio;   // Near-zero delay — drives IDDR only
wire adc_dco_buffered; // BUFG output — drives fabric logic

BUFIO bufio_dco (
    .I(adc_dco),
    .O(adc_dco_bufio)
);

// MMCME2 jitter-cleaning wrapper replaces the direct BUFG.
// The PLL feedback loop attenuates input jitter from ~50 ps to ~20-30 ps,
// reducing clock uncertainty and improving WNS on the 400 MHz CIC path.
wire mmcm_locked;

adc_clk_mmcm mmcm_inst (
    .clk_in       (adc_dco),          // 400 MHz from IBUFDS output
    .reset_n      (reset_n),
    .clk_400m_out (adc_dco_buffered), // Jitter-cleaned 400 MHz on BUFG
    .mmcm_locked  (mmcm_locked)
);
assign adc_dco_bufg = adc_dco_buffered;

// IDDR for capturing DDR data
wire [7:0] adc_data_rise;  // Data on rising edge (BUFIO domain)
wire [7:0] adc_data_fall;  // Data on falling edge (BUFIO domain)

genvar j;
generate
    for (j = 0; j < 8; j = j + 1) begin : iddr_gen
        IDDR #(
            .DDR_CLK_EDGE("SAME_EDGE_PIPELINED"),
            .INIT_Q1(1'b0),
            .INIT_Q2(1'b0),
            .SRTYPE("SYNC")
        ) iddr_inst (
            .Q1(adc_data_rise[j]),   // Rising edge data
            .Q2(adc_data_fall[j]),   // Falling edge data
            .C(adc_dco_bufio),       // BUFIO clock (near-zero insertion delay)
            .CE(1'b1),
            .D(adc_data[j]),
            .R(1'b0),
            .S(1'b0)
        );
    end
endgenerate

// ============================================================================
// Re-register IDDR outputs into BUFG domain
// IDDR with SAME_EDGE_PIPELINED produces outputs stable for a full clock cycle.
// BUFIO and BUFG are derived from the same source (adc_dco), so they are
// frequency-matched. This single register stage transfers from IOB (BUFIO)
// to fabric (BUFG) with guaranteed timing.
// ============================================================================
reg [7:0] adc_data_rise_bufg;
reg [7:0] adc_data_fall_bufg;

always @(posedge adc_dco_buffered) begin
    adc_data_rise_bufg <= adc_data_rise;
    adc_data_fall_bufg <= adc_data_fall;
end

// Combine rising and falling edge data to get 400MSPS stream
reg [7:0] adc_data_400m_reg;
reg adc_data_valid_400m_reg;
reg dco_phase;

// ── Reset synchronizer ────────────────────────────────────────
// reset_n comes from the 100 MHz sys_clk domain.  Assertion (going low)
// is asynchronous and safe — the FFs enter reset instantly.  De-assertion
// (going high) must be synchronised to adc_dco_buffered to avoid
// metastability.  This is the classic "async assert, sync de-assert" pattern.
//
// mmcm_locked gates de-assertion: the 400 MHz domain stays in reset until
// the MMCM PLL has locked and the jitter-cleaned clock is stable.
(* ASYNC_REG = "TRUE" *) reg [1:0] reset_sync_400m;
wire reset_n_400m;
wire reset_n_gated = reset_n & mmcm_locked;

always @(posedge adc_dco_buffered or negedge reset_n_gated) begin
    if (!reset_n_gated)
        reset_sync_400m <= 2'b00;           // async assert (or MMCM not locked)
    else
        reset_sync_400m <= {reset_sync_400m[0], 1'b1};  // sync de-assert
end
assign reset_n_400m = reset_sync_400m[1];

always @(posedge adc_dco_buffered or negedge reset_n_400m) begin
    if (!reset_n_400m) begin
        adc_data_400m_reg <= 8'b0;
        adc_data_valid_400m_reg <= 1'b0;
        dco_phase <= 1'b0;
    end else begin
        dco_phase <= ~dco_phase;
        
        if (dco_phase) begin
            // Output falling edge data (completes the 400MSPS stream)
            adc_data_400m_reg <= adc_data_fall_bufg;
        end else begin
            // Output rising edge data
            adc_data_400m_reg <= adc_data_rise_bufg;
        end
        
        adc_data_valid_400m_reg <= 1'b1; // Always valid when ADC is running
    end
end

assign adc_data_400m = adc_data_400m_reg;
assign adc_data_valid_400m = adc_data_valid_400m_reg;

endmodule