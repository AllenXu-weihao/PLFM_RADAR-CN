`timescale 1ns / 1ps

/**
 * mti_canceller.v — 动目标指示（MTI）杂波抑制滤波器
 *
 * 【中文功能概述】
 * 可配置 2/3 脉冲对消器，用于消除地物杂波（静止目标回波）。
 *
 * 【优化改进】新增 3 脉冲对消选项（PULSES=3），杂波抑制能力提升一倍。
 *   - 2 脉冲：H(z) = 1 - z^{-1}        （6 dB/Hz 杂波改善）
 *   - 3 脉冲：H(z) = 1 - 2z^{-1} + z^{-2}（12 dB/Hz 杂波改善，陷波更陡峭）
 *
 * 【信号链位置】
 *   距离 Bin 抽取器 → [MTI 对消器] → Doppler 处理器
 *
 * 【算法】
 *   2 脉冲模式：mti_out[n] = current[n] - previous[n]
 *   3 脉冲模式：mti_out[n] = current[n] - 2*previous[n] + preprevious[n]
 *
 * 前一次啁啾的距离 bin 数据存储在 BRAM 中。
 * 复位或使能后的最初几个啁啾输出为零（静音）。
 *
 * 当 mti_enable=0 时，模块为透明直通。
 *
 * 【资源消耗】
 *   2 脉冲：2 BRAM18 (64×16 I+Q)，~30 LUTs，~40 FFs
 *   3 脉冲：4 BRAM18 (64×16 I+Q × 2)，~60 LUTs，~60 FFs
 *
 * Clock domain: clk (100 MHz)
 */

module mti_canceller #(
    parameter NUM_RANGE_BINS = 64,    // 距离 bin 数量
    parameter DATA_WIDTH     = 16,    // 数据位宽
    parameter PULSES         = 2      // 脉冲数：2=二脉冲对消，3=三脉冲对消（优化改进）
) (
    input wire clk,                   // 系统时钟 100MHz
    input wire reset_n,               // 异步复位（低有效）

    // ========== 输入（来自距离 Bin 抽取器）==========
    input wire signed [DATA_WIDTH-1:0] range_i_in,   // I 通道数据
    input wire signed [DATA_WIDTH-1:0] range_q_in,   // Q 通道数据
    input wire                         range_valid_in,  // 数据有效
    input wire [5:0]                   range_bin_in,    // 当前 bin 索引

    // ========== 输出（至 Doppler 处理器）==========
    output reg signed [DATA_WIDTH-1:0] range_i_out,    // MTI 后 I 通道
    output reg signed [DATA_WIDTH-1:0] range_q_out,    // MTI 后 Q 通道
    output reg                         range_valid_out, // 输出有效
    output reg [5:0]                   range_bin_out,   // bin 索引直通

    // ========== 配置（Configuration）==========
    input wire mti_enable,             // 1=MTI 激活，0=透明直通

    // ========== 状态输出（Status）==========
    output reg mti_first_chirp         // 首个啁啍期间为 1（输出静音）
);

// ============================================================================
// 前一次啁啍缓冲区（PREVIOUS CHIRP BUFFER）
// 64 × 16bit I + 64 × 16bit Q = 存储上一啁啍的所有距离 bin 数据
// 【优化】3 脉冲模式增加前前次缓冲区，实现 H(z)=1-2z^{-1}+z^{-2}
// ============================================================================

reg signed [DATA_WIDTH-1:0] prev_i [0:NUM_RANGE_BINS-1];   // 上一次 I
reg signed [DATA_WIDTH-1:0] prev_q [0:NUM_RANGE_BINS-1];   // 上一次 Q

// 【优化】3 脉冲对消用的前前次缓冲区
reg signed [DATA_WIDTH-1:0] prev2_i [0:NUM_RANGE_BINS-1];  // 前前次 I
reg signed [DATA_WIDTH-1:0] prev2_q [0:NUM_RANGE_BINS-1];  // 前前次 Q

// 有效啁啍计数（0, 1, 或 2+），用于判断是否有足够历史数据做对消
reg [1:0] valid_chirp_count;

// ============================================================================
// MTI 对消处理逻辑（MTI PROCESSING）
// ============================================================================

// 读取前一次啁啍数据（组合逻辑）
wire signed [DATA_WIDTH-1:0] prev_i_rd = prev_i[range_bin_in];
wire signed [DATA_WIDTH-1:0] prev_q_rd = prev_q[range_bin_in];

// 【优化】读取前前次啁啍数据（3 脉冲对消用）
wire signed [DATA_WIDTH-1:0] prev2_i_rd = prev2_i[range_bin_in];
wire signed [DATA_WIDTH-1:0] prev2_q_rd = prev2_q[range_bin_in];

// 计算差分（带饱和保护）
// 2 脉冲：diff = current - previous
// 3 脉冲：diff = current - 2*previous + preprevious
wire signed [DATA_WIDTH:0] diff_i_full = {range_i_in[DATA_WIDTH-1], range_i_in}
                                        - {prev_i_rd[DATA_WIDTH-1], prev_i_rd};
wire signed [DATA_WIDTH:0] diff_q_full = {range_q_in[DATA_WIDTH-1], range_q_in}
                                        - {prev_q_rd[DATA_WIDTH-1], prev_q_rd};

// 【优化】3 脉冲差分计算
// diff3 = (current - previous) + (preprevious - previous) = diff2 + (prev2 - prev)
wire signed [DATA_WIDTH+1:0] prev2_minus_prev_i = {prev2_i_rd[DATA_WIDTH-1], prev2_i_rd}
                                                 - {prev_i_rd[DATA_WIDTH-1], prev_i_rd};
wire signed [DATA_WIDTH+1:0] prev2_minus_prev_q = {prev2_q_rd[DATA_WIDTH-1], prev2_q_rd}
                                                 - {prev_q_rd[DATA_WIDTH-1], prev_q_rd};
wire signed [DATA_WIDTH+1:0] diff3_i_full = {diff_i_full[DATA_WIDTH], diff_i_full}
                                           + prev2_minus_prev_i;
wire signed [DATA_WIDTH+1:0] diff3_q_full = {diff_q_full[DATA_WIDTH], diff_q_full}
                                           + prev2_minus_prev_q;

// Select between 2-pulse and 3-pulse results
wire signed [DATA_WIDTH+1:0] diff_i_sel = (PULSES == 3) ? diff3_i_full
                                          : {{2{diff_i_full[DATA_WIDTH]}}, diff_i_full};
wire signed [DATA_WIDTH+1:0] diff_q_sel = (PULSES == 3) ? diff3_q_full
                                          : {{2{diff_q_full[DATA_WIDTH]}}, diff_q_full};

// Saturate to DATA_WIDTH bits
// [OPT] Updated to handle wider intermediate results for 3-pulse mode
wire signed [DATA_WIDTH-1:0] diff_i_sat;
wire signed [DATA_WIDTH-1:0] diff_q_sat;

assign diff_i_sat = (diff_i_sel > $signed({{(3-DATA_WIDTH+DATA_WIDTH){1'b0}}, {(DATA_WIDTH-1){1'b1}}}))
                  ? $signed({1'b0, {(DATA_WIDTH-1){1'b1}}})           // +max
                  : (diff_i_sel < $signed({1'b1, {(DATA_WIDTH){1'b0}}}))
                  ? $signed({1'b1, {(DATA_WIDTH-1){1'b0}}})           // -max
                  : diff_i_sel[DATA_WIDTH-1:0];

assign diff_q_sat = (diff_q_sel > $signed({{(3-DATA_WIDTH+DATA_WIDTH){1'b0}}, {(DATA_WIDTH-1){1'b1}}}))
                  ? $signed({1'b0, {(DATA_WIDTH-1){1'b1}}})
                  : (diff_q_sel < $signed({1'b1, {(DATA_WIDTH){1'b0}}}))
                  ? $signed({1'b1, {(DATA_WIDTH-1){1'b0}}})
                  : diff_q_sel[DATA_WIDTH-1:0];

// ============================================================================
// 【核心】MTI 主处理逻辑（MAIN LOGIC）
// 【优化】支持 2 脉冲和 3 脉冲对消模式
// ============================================================================
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        range_i_out     <= {DATA_WIDTH{1'b0}};    // 复位：输出清零
        range_q_out     <= {DATA_WIDTH{1'b0}};
        range_valid_out <= 1'b0;
        range_bin_out   <= 6'd0;
        valid_chirp_count <= 2'd0;
        mti_first_chirp <= 1'b1;                   // 标记为首啁啍（输出静音）
    end else begin
        // 默认：无有效输出
        range_valid_out <= 1'b0;

        if (range_valid_in) begin
            // 始终将当前样本存为"前一次"，供下次啁啾对消使用
            // 【优化】3 脉冲模式：prev → prev2 移位，当前 → prev
            if (PULSES == 3) begin
                prev2_i[range_bin_in] <= prev_i[range_bin_in];     // prev → prev2
                prev2_q[range_bin_in] <= prev_q[range_bin_in];
            end
            prev_i[range_bin_in] <= range_i_in;                    // 当前 → prev
            prev_q[range_bin_in] <= range_q_in;

            // 输出路径：bin 索引直通
            range_bin_out <= range_bin_in;

            if (!mti_enable) begin
                // ========== 直通模式：不做 MTI 处理 ==========
                range_i_out     <= range_i_in;
                range_q_out     <= range_q_in;
                range_valid_out <= 1'b1;
                // MTI 禁用时重置状态
                valid_chirp_count <= 2'd0;
                mti_first_chirp <= 1'b1;
            end else if (valid_chirp_count < (PULSES - 1)) begin
                // ========== 历史数据不足：静音输出 ==========
                // 2 脉冲需 1 个前次啁啍（count 到达 1）
                // 3 脉冲需 2 个前次啁啍（count 到达 2）
                range_i_out     <= {DATA_WIDTH{1'b0}};
                range_q_out     <= {DATA_WIDTH{1'b0}};
                range_valid_out <= 1'b1;

                // 当该啁啍的最后一个 bin 处理完后，递增计数
                if (range_bin_in == NUM_RANGE_BINS - 1) begin
                    valid_chirp_count <= valid_chirp_count + 1;
                    if (valid_chirp_count == PULSES - 2) begin
                        mti_first_chirp <= 1'b0;      // 下一个啁啍开始正常 MTI 输出
                    end
                end
            end else begin
                // ========== 正常 MTI 模式：前次 - 当前 = 运动目标 ==========
                range_i_out     <= diff_i_sat;         // 对消后 I 输出
                range_q_out     <= diff_q_sat;         // 对消后 Q 输出
                range_valid_out <= 1'b1;
            end
        end
    end
end

// ============================================================================
// MEMORY INITIALIZATION (simulation only)
// ============================================================================
`ifdef SIMULATION
integer init_k;
initial begin
    for (init_k = 0; init_k < NUM_RANGE_BINS; init_k = init_k + 1) begin
        prev_i[init_k] = 0;
        prev_q[init_k] = 0;
        prev2_i[init_k] = 0;  // [OPT] 3-pulse buffer init
        prev2_q[init_k] = 0;
    end
end
`endif

endmodule
