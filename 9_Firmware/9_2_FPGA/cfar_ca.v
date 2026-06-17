`timescale 1ns / 1ps

/**
 * cfar_ca.v
 *
 * Cell-Averaging CFAR (Constant False Alarm Rate) Detector
 * for the AERIS-10 phased-array radar.
 *
 * Replaces the simple magnitude threshold detector in radar_system_top.v
 * (lines 474-514) with a proper adaptive-threshold CFAR algorithm.
 *
 * Architecture:
 *   Phase 1 (BUFFER): As Doppler processor outputs arrive, compute |I|+|Q|
 *     magnitude and store in BRAM. Address = {range_bin, doppler_bin}.
 *     When CFAR is disabled, applies simple threshold pass-through.
 *
 *   Phase 2 (CFAR): After frame_complete pulse from Doppler processor,
 *     process each Doppler column independently:
 *     a) Read 64 magnitudes from BRAM for one Doppler bin (ST_COL_LOAD)
 *     b) Compute initial sliding window sums (ST_CFAR_INIT)
 *     c) Slide CUT through all 64 range bins:
 *        - 3 sub-cycles per CUT:
 *          ST_CFAR_THR: register noise_sum (mode select + cross-multiply)
 *          ST_CFAR_MUL: compute alpha * noise_sum_reg in DSP
 *          ST_CFAR_CMP: compare CUT magnitude against threshold + update window
 *     d) Advance to next Doppler column (ST_COL_NEXT)
 *
 * CFAR Modes (cfg_cfar_mode):
 *   2'b00 = CA-CFAR:  noise = leading_sum + lagging_sum
 *   2'b01 = GO-CFAR:  noise = max(leading_sum * lag_cnt, lagging_sum * lead_cnt)
 *                      normalized — picks larger average
 *   2'b10 = SO-CFAR:  noise = min(leading_sum * lag_cnt, lagging_sum * lead_cnt)
 *   2'b11 = Reserved (falls back to CA-CFAR)
 *
 * Threshold computation:
 *   threshold = (alpha * noise_sum) >> ALPHA_FRAC_BITS
 *   Host sets alpha in Q4.4 fixed-point, pre-compensated for training cell count.
 *   Example: for T=8 cells per side (16 total), desired Pfa=1e-4:
 *     alpha_statistical ≈ 4.88
 *     alpha_fpga = alpha_statistical / 16 = 0.305 → Q4.4 ≈ 0x05
 *   Or host can set alpha per training cell if it accounts for count.
 *
 * Edge handling:
 *   At range boundaries where the full window doesn't fit, only available
 *   training cells are used. The noise estimate naturally reduces, raising
 *   false alarm rate at edges — acceptable for radar (edge bins are
 *   typically clutter).
 *
 * Timing:
 *   Phase 2 takes ~(66 + T + 3*64) * 32 ≈ 8500 cycles per frame @ 100 MHz
 *   = 85 µs. Frame period @ PRF=1932 Hz, 32 chirps = 16.6 ms. Fits easily.
 *   (3 cycles per CUT due to pipeline: THR → MUL → CMP)
 *
 * Resources:
 *   - 1 BRAM18K for magnitude buffer (2048 x 17 bits)
 *   - 1 DSP48 for alpha multiply
 *   - ~300 LUTs for FSM + sliding window + comparators
 *
 * Clock domain: clk (100 MHz, same as Doppler processor)
 */

// ============================================================================
// 中文摘要（Chinese Summary）
// ============================================================================
// CFAR-CA 自适应门限检测器
// 
// 功能：
// 使用单元平均 CFAR 算法替代简单的幅度门限检测。
// 通过在检测单元两侧设置保护单元和参考窗，估计局部噪声功率，自适应设置检测门限。
// 
// 两种处理阶段：
// - 阶段 1（BUFFER）：存储 Doppler 处理器输出的 |I|+|Q| 幅值到 BRAM
// - 阶段 2（CFAR）：帧完成后，对每个 Doppler 列独立处理，滑动窗口噪声估计
// 
// 三种 CFAR 模式（cfg_cfar_mode）：
// - 2'b00 = CA-CFAR：平均前后窗噪声功率（标准单元平均）
// - 2'b01 = GO-CFAR：选择平均功率大的一侧（多目标场景）
// - 2'b10 = SO-CFAR：选择平均功率小的一侧（边带干扰场景）
// 
// 幅值近似优化：使用 max + min/2 近似计算幅值，节省平方根运算（误差约 5%）
// 
// 时钟域：clk (100 MHz)
// 资源：1 BRAM18K + 1 DSP48 + ~300 LUTs
// ============================================================================

module cfar_ca #(
    parameter NUM_RANGE_BINS   = 64,
    parameter NUM_DOPPLER_BINS = 32,
    parameter MAG_WIDTH        = 17,
    parameter ALPHA_WIDTH      = 8,
    parameter MAX_GUARD        = 8,
    parameter MAX_TRAIN        = 16
) (
    input wire clk,
    input wire reset_n,

    // ========== DOPPLER PROCESSOR INPUTS ==========
    input wire [31:0] doppler_data,
    input wire        doppler_valid,
    input wire [4:0]  doppler_bin_in,
    input wire [5:0]  range_bin_in,
    input wire        frame_complete,

    // ========== CONFIGURATION ==========
    input wire [3:0]  cfg_guard_cells,
    input wire [4:0]  cfg_train_cells,
    input wire [ALPHA_WIDTH-1:0] cfg_alpha,
    input wire [1:0]  cfg_cfar_mode,
    input wire        cfg_cfar_enable,
    input wire [15:0] cfg_simple_threshold,

    // ========== DETECTION OUTPUTS ==========
    output reg        detect_flag,
    output reg        detect_valid,
    output reg [5:0]  detect_range,
    output reg [4:0]  detect_doppler,
    output reg [MAG_WIDTH-1:0] detect_magnitude,
    output reg [MAG_WIDTH-1:0] detect_threshold,

    // ========== STATUS ==========
    output reg [15:0] detect_count,
    output wire       cfar_busy,
    output reg [7:0]  cfar_status
);

// ============================================================================
// INTERNAL PARAMETERS
// ============================================================================
localparam TOTAL_CELLS = NUM_RANGE_BINS * NUM_DOPPLER_BINS;
localparam ADDR_WIDTH  = 11;
localparam COL_BITS    = 5;
localparam ROW_BITS    = 6;
localparam SUM_WIDTH   = MAG_WIDTH + 6;  // 23 bits: sum of up to 64 magnitudes
localparam PROD_WIDTH  = SUM_WIDTH + ALPHA_WIDTH;  // 31 bits
localparam ALPHA_FRAC_BITS = 4;  // Q4.4

// ============================================================================
// FSM STATES
// ============================================================================
localparam [3:0] ST_IDLE       = 4'd0,
                 ST_BUFFER     = 4'd1,
                 ST_COL_LOAD   = 4'd2,
                 ST_CFAR_INIT  = 4'd3,
                 ST_CFAR_THR   = 4'd4,  // Register noise_sum (mode select + cross-multiply)
                 ST_CFAR_MUL   = 4'd8,  // Compute alpha * noise_sum_reg in DSP
                 ST_CFAR_CMP   = 4'd5,  // Compare + update window
                 ST_COL_NEXT   = 4'd6,
                 ST_DONE       = 4'd7;

reg [3:0] state;
assign cfar_busy = (state != ST_IDLE);

// ============================================================================
// MAGNITUDE COMPUTATION (combinational)
// [OPT-IMPROVE] Alpha-max-plus-beta-min magnitude approximation.
// Original: |I|+|Q| (overestimates by up to 41.4% at 45°)
// Optimized: max(|I|,|Q|) + min(|I|,|Q|)/2 (max 11.8% error)
// This improves CFAR threshold accuracy, reducing false alarms at
// diagonal positions and improving detection of off-axis targets.
// Cost: 1 extra adder + 1 shifter (negligible vs BRAM/DSP budget).
// ============================================================================
wire signed [15:0] dop_i = doppler_data[15:0];
wire signed [15:0] dop_q = doppler_data[31:16];
wire [15:0] abs_i = dop_i[15] ? (~dop_i + 16'd1) : dop_i;
wire [15:0] abs_q = dop_q[15] ? (~dop_q + 16'd1) : dop_q;
wire [15:0] mag_max = (abs_i > abs_q) ? abs_i : abs_q;
wire [15:0] mag_min = (abs_i > abs_q) ? abs_q : abs_i;
wire [MAG_WIDTH-1:0] cur_mag = {1'b0, mag_max} + {2'b0, mag_min[15:1]}; // max + min/2

// ============================================================================
// MAGNITUDE BRAM (2048 x 17 bits)
// ============================================================================
reg                    mag_we;
reg [ADDR_WIDTH-1:0]   mag_waddr;
reg [MAG_WIDTH-1:0]    mag_wdata;
reg [ADDR_WIDTH-1:0]   mag_raddr;
reg [MAG_WIDTH-1:0]    mag_rdata;

(* ram_style = "block" *) reg [MAG_WIDTH-1:0] mag_mem [0:TOTAL_CELLS-1];

always @(posedge clk) begin
    if (mag_we)
        mag_mem[mag_waddr] <= mag_wdata;
    mag_rdata <= mag_mem[mag_raddr];
end

// ============================================================================
// COLUMN LINE BUFFER (64 x 17 bits — distributed RAM)
// ============================================================================
reg [MAG_WIDTH-1:0] col_buf [0:NUM_RANGE_BINS-1];
reg [ROW_BITS:0] col_load_idx;

// ============================================================================
// SLIDING WINDOW STATE
// ============================================================================
reg [SUM_WIDTH-1:0] leading_sum;
reg [SUM_WIDTH-1:0] lagging_sum;
reg [ROW_BITS:0]    leading_count;
reg [ROW_BITS:0]    lagging_count;
reg [ROW_BITS:0]    cut_idx;
reg [COL_BITS-1:0]  col_idx;

// Registered config (captured at frame start)
reg [3:0]  r_guard;
reg [4:0]  r_train;
reg [ALPHA_WIDTH-1:0] r_alpha;
reg [1:0]  r_mode;
reg        r_enable;
reg [15:0] r_simple_thr;

// Threshold pipeline registers
reg [SUM_WIDTH-1:0]  noise_sum_reg;   // Stage 1: registered noise_sum_comb output
reg [PROD_WIDTH-1:0] noise_product;   // Stage 2: alpha * noise_sum_reg
reg [MAG_WIDTH-1:0]  adaptive_thr;

// Init counter for computing initial lagging sum
reg [ROW_BITS:0] init_idx;

// ============================================================================
// SLIDING WINDOW DELTA COMPUTATION (combinational)
// ============================================================================
// Compute net delta to leading_sum and lagging_sum when CUT advances by 1.
// All deltas computed combinationally, applied as a single NBA per register.

// Indices of cells entering/leaving the window when CUT moves from k to k+1:
//   Leading: new training cell at index k+1-G-1 = k-G (was closest guard cell)
//            cell falling off at index k+1-G-T-1 = k-G-T
//   Lagging: cell leaving at index k+G+1 (enters guard zone)
//            new cell entering at index k+1+G+T (at far end)

wire signed [ROW_BITS+1:0] lead_add_idx = $signed({1'b0, cut_idx}) - $signed({1'b0, r_guard});
wire signed [ROW_BITS+1:0] lead_rem_idx = $signed({1'b0, cut_idx}) - $signed({1'b0, r_guard}) - $signed({1'b0, r_train});
wire signed [ROW_BITS+1:0] lag_rem_idx  = $signed({1'b0, cut_idx}) + $signed({1'b0, r_guard}) + 1;
wire signed [ROW_BITS+1:0] lag_add_idx  = $signed({1'b0, cut_idx}) + 1 + $signed({1'b0, r_guard}) + $signed({1'b0, r_train});

wire lead_add_valid = (lead_add_idx >= 0) && (lead_add_idx < NUM_RANGE_BINS);
wire lead_rem_valid = (lead_rem_idx >= 0) && (lead_rem_idx < NUM_RANGE_BINS);
wire lag_rem_valid  = (lag_rem_idx  >= 0) && (lag_rem_idx  < NUM_RANGE_BINS);
wire lag_add_valid  = (lag_add_idx  >= 0) && (lag_add_idx  < NUM_RANGE_BINS);

// Safe col_buf read with bounds checking (combinational)
wire [MAG_WIDTH-1:0] lead_add_val = lead_add_valid ? col_buf[lead_add_idx[ROW_BITS-1:0]] : {MAG_WIDTH{1'b0}};
wire [MAG_WIDTH-1:0] lead_rem_val = lead_rem_valid ? col_buf[lead_rem_idx[ROW_BITS-1:0]] : {MAG_WIDTH{1'b0}};
wire [MAG_WIDTH-1:0] lag_rem_val  = lag_rem_valid  ? col_buf[lag_rem_idx[ROW_BITS-1:0]]  : {MAG_WIDTH{1'b0}};
wire [MAG_WIDTH-1:0] lag_add_val  = lag_add_valid  ? col_buf[lag_add_idx[ROW_BITS-1:0]]  : {MAG_WIDTH{1'b0}};

// Net deltas
wire signed [SUM_WIDTH:0] lead_delta = (lead_add_valid ? $signed({1'b0, lead_add_val}) : 0)
                                      - (lead_rem_valid ? $signed({1'b0, lead_rem_val}) : 0);
wire signed [1:0] lead_cnt_delta = (lead_add_valid ? 1 : 0) - (lead_rem_valid ? 1 : 0);

wire signed [SUM_WIDTH:0] lag_delta = (lag_add_valid ? $signed({1'b0, lag_add_val}) : 0)
                                     - (lag_rem_valid ? $signed({1'b0, lag_rem_val}) : 0);
wire signed [1:0] lag_cnt_delta = (lag_add_valid ? 1 : 0) - (lag_rem_valid ? 1 : 0);

// ============================================================================
// NOISE ESTIMATE COMPUTATION (combinational for CFAR mode selection)
// ============================================================================
reg [SUM_WIDTH-1:0] noise_sum_comb;

always @(*) begin
    case (r_mode)
    2'b00, 2'b11: begin  // CA-CFAR
        noise_sum_comb = leading_sum + lagging_sum;
    end
    2'b01: begin  // GO-CFAR: pick sum from side with greater average
        if (leading_count > 0 && lagging_count > 0) begin
            // leading_avg > lagging_avg ↔ leading_sum * lagging_count > lagging_sum * leading_count
            if (leading_sum * lagging_count > lagging_sum * leading_count)
                noise_sum_comb = leading_sum;
            else
                noise_sum_comb = lagging_sum;
        end else if (leading_count > 0)
            noise_sum_comb = leading_sum;
        else
            noise_sum_comb = lagging_sum;
    end
    2'b10: begin  // SO-CFAR: pick sum from side with smaller average
        if (leading_count > 0 && lagging_count > 0) begin
            if (leading_sum * lagging_count < lagging_sum * leading_count)
                noise_sum_comb = leading_sum;
            else
                noise_sum_comb = lagging_sum;
        end else if (leading_count > 0)
            noise_sum_comb = leading_sum;
        else
            noise_sum_comb = lagging_sum;
    end
    default:
        noise_sum_comb = leading_sum + lagging_sum;
    endcase
end

// ============================================================================
// MAIN FSM
// ============================================================================
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        state          <= ST_IDLE;
        detect_flag    <= 1'b0;
        detect_valid   <= 1'b0;
        detect_range   <= 6'd0;
        detect_doppler <= 5'd0;
        detect_magnitude <= {MAG_WIDTH{1'b0}};
        detect_threshold <= {MAG_WIDTH{1'b0}};
        detect_count   <= 16'd0;
        cfar_status    <= 8'd0;
        mag_we         <= 1'b0;
        mag_waddr      <= {ADDR_WIDTH{1'b0}};
        mag_wdata      <= {MAG_WIDTH{1'b0}};
        mag_raddr      <= {ADDR_WIDTH{1'b0}};
        col_load_idx   <= 0;
        col_idx        <= 0;
        cut_idx        <= 0;
        leading_sum    <= 0;
        lagging_sum    <= 0;
        leading_count  <= 0;
        lagging_count  <= 0;
        init_idx       <= 0;
        noise_sum_reg  <= 0;
        noise_product  <= 0;
        adaptive_thr   <= 0;
        r_guard        <= 4'd2;
        r_train        <= 5'd8;
        r_alpha        <= 8'h30;
        r_mode         <= 2'b00;
        r_enable       <= 1'b0;
        r_simple_thr   <= 16'd10000;
    end else begin
        // Defaults: clear one-shot outputs
        detect_valid <= 1'b0;
        detect_flag  <= 1'b0;
        mag_we       <= 1'b0;

        case (state)
        // ================================================================
        // ST_IDLE: Wait for first Doppler output
        // ================================================================
        ST_IDLE: begin
            cfar_status <= 8'd0;

            if (doppler_valid) begin
                // Capture configuration at frame start
                r_guard      <= cfg_guard_cells;
                r_train      <= (cfg_train_cells == 0) ? 5'd1 : cfg_train_cells;
                r_alpha      <= cfg_alpha;
                r_mode       <= cfg_cfar_mode;
                r_enable     <= cfg_cfar_enable;
                r_simple_thr <= cfg_simple_threshold;

                // Buffer first sample
                mag_we    <= 1'b1;
                mag_waddr <= {range_bin_in, doppler_bin_in};
                mag_wdata <= cur_mag;

                // Simple threshold pass-through when CFAR disabled
                if (!cfg_cfar_enable) begin
                    detect_flag  <= (cur_mag > {1'b0, cfg_simple_threshold});
                    detect_valid <= 1'b1;
                    detect_range     <= range_bin_in;
                    detect_doppler   <= doppler_bin_in;
                    detect_magnitude <= cur_mag;
                    detect_threshold <= {1'b0, cfg_simple_threshold};
                    if (cur_mag > {1'b0, cfg_simple_threshold})
                        detect_count <= detect_count + 1;
                end

                state <= ST_BUFFER;
            end
        end

        // ================================================================
        // ST_BUFFER: Store magnitudes until frame complete
        // ================================================================
        ST_BUFFER: begin
            cfar_status <= {4'd1, 4'd0};

            if (doppler_valid) begin
                mag_we    <= 1'b1;
                mag_waddr <= {range_bin_in, doppler_bin_in};
                mag_wdata <= cur_mag;

                if (!r_enable) begin
                    detect_flag  <= (cur_mag > {1'b0, r_simple_thr});
                    detect_valid <= 1'b1;
                    detect_range     <= range_bin_in;
                    detect_doppler   <= doppler_bin_in;
                    detect_magnitude <= cur_mag;
                    detect_threshold <= {1'b0, r_simple_thr};
                    if (cur_mag > {1'b0, r_simple_thr})
                        detect_count <= detect_count + 1;
                end
            end

            if (frame_complete) begin
                if (r_enable) begin
                    col_idx      <= 0;
                    col_load_idx <= 0;
                    mag_raddr    <= {6'd0, 5'd0};
                    state        <= ST_COL_LOAD;
                end else begin
                    state <= ST_DONE;
                end
            end
        end

        // ================================================================
        // ST_COL_LOAD: Read one Doppler column from BRAM
        // ================================================================
        // BRAM has 1-cycle read latency. Pipeline: present addr cycle N,
        // capture data cycle N+1.
        ST_COL_LOAD: begin
            cfar_status <= {4'd2, 1'b0, col_idx[2:0]};

            if (col_load_idx == 0) begin
                // First address already presented, advance to range=1
                mag_raddr    <= {6'd1, col_idx};
                col_load_idx <= 1;
            end else if (col_load_idx <= NUM_RANGE_BINS) begin
                // Capture previous read
                col_buf[col_load_idx - 1] <= mag_rdata;

                if (col_load_idx < NUM_RANGE_BINS) begin
                    mag_raddr <= {col_load_idx[ROW_BITS-1:0] + 6'd1, col_idx};
                end

                col_load_idx <= col_load_idx + 1;
            end

            if (col_load_idx == NUM_RANGE_BINS + 1) begin
                // Column fully loaded → initialize CFAR window
                state         <= ST_CFAR_INIT;
                init_idx      <= 0;
                leading_sum   <= 0;
                lagging_sum   <= 0;
                leading_count <= 0;
                lagging_count <= 0;
                cut_idx       <= 0;
            end
        end

        // ================================================================
        // ST_CFAR_INIT: Compute initial window sums for CUT=0
        // ================================================================
        // CUT=0 has no leading cells. Lagging cells are at
        // indices [guard+1 .. guard+train] (if they exist).
        // Iterate one training cell per cycle.
        ST_CFAR_INIT: begin
            cfar_status <= {4'd3, 1'b0, col_idx[2:0]};

            if (init_idx < r_train) begin
                if ((r_guard + 1 + init_idx) < NUM_RANGE_BINS) begin
                    lagging_sum   <= lagging_sum + col_buf[r_guard + 1 + init_idx];
                    lagging_count <= lagging_count + 1;
                end
                init_idx <= init_idx + 1;
            end else begin
                // Initial sums ready → begin CFAR sliding
                state <= ST_CFAR_THR;
            end
        end

        // ================================================================
        // ST_CFAR_THR: Register noise estimate (mode select + cross-multiply)
        // ================================================================
        // Pipeline stage 1: register the combinational noise_sum_comb
        // output. This breaks the critical path:
        //   leading_sum → cross-multiply (GO/SO) → mux → alpha*noise DSP
        // into two shorter paths:
        //   Cycle 1: leading_sum → cross-multiply → mux → noise_sum_reg
        //   Cycle 2: noise_sum_reg → alpha * noise_sum_reg → noise_product
        ST_CFAR_THR: begin
            cfar_status <= {4'd4, 1'b0, col_idx[2:0]};

            noise_sum_reg <= noise_sum_comb;
            state <= ST_CFAR_MUL;
        end

        // ================================================================
        // ST_CFAR_MUL: Compute alpha * noise_sum_reg in DSP
        // ================================================================
        // Pipeline stage 2: multiply registered noise sum by alpha.
        // This is a clean registered-input → DSP path.
        ST_CFAR_MUL: begin
            cfar_status <= {4'd4, 1'b1, col_idx[2:0]};

            noise_product <= r_alpha * noise_sum_reg;
            state <= ST_CFAR_CMP;
        end

        // ================================================================
        // ST_CFAR_CMP: Compare CUT against threshold + update window
        // ================================================================
        ST_CFAR_CMP: begin
            cfar_status <= {4'd5, 1'b0, col_idx[2:0]};

            // Threshold = noise_product >> ALPHA_FRAC_BITS
            // Saturate to MAG_WIDTH bits
            if (noise_product[PROD_WIDTH-1:ALPHA_FRAC_BITS+MAG_WIDTH] != 0)
                adaptive_thr <= {MAG_WIDTH{1'b1}};  // Saturate
            else
                adaptive_thr <= noise_product[ALPHA_FRAC_BITS +: MAG_WIDTH];

            // Output detection result
            detect_magnitude <= col_buf[cut_idx[ROW_BITS-1:0]];
            detect_range     <= cut_idx[ROW_BITS-1:0];
            detect_doppler   <= col_idx;
            detect_valid     <= 1'b1;

            // Compare: threshold computed this cycle from noise_product
            begin : threshold_compare
                reg [MAG_WIDTH-1:0] thr_val;
                if (noise_product[PROD_WIDTH-1:ALPHA_FRAC_BITS+MAG_WIDTH] != 0)
                    thr_val = {MAG_WIDTH{1'b1}};
                else
                    thr_val = noise_product[ALPHA_FRAC_BITS +: MAG_WIDTH];

                detect_threshold <= thr_val;

                if (col_buf[cut_idx[ROW_BITS-1:0]] > thr_val) begin
                    detect_flag  <= 1'b1;
                    detect_count <= detect_count + 1;
                end
            end

            // Update sliding window for next CUT
            if (cut_idx < NUM_RANGE_BINS - 1) begin
                // Apply pre-computed deltas (single NBA per register)
                leading_sum   <= $unsigned($signed({1'b0, leading_sum}) + lead_delta);
                leading_count <= $unsigned($signed({1'b0, leading_count}) + {{(ROW_BITS){lead_cnt_delta[1]}}, lead_cnt_delta});
                lagging_sum   <= $unsigned($signed({1'b0, lagging_sum}) + lag_delta);
                lagging_count <= $unsigned($signed({1'b0, lagging_count}) + {{(ROW_BITS){lag_cnt_delta[1]}}, lag_cnt_delta});

                cut_idx <= cut_idx + 1;
                state   <= ST_CFAR_THR;
            end else begin
                state <= ST_COL_NEXT;
            end
        end

        // ================================================================
        // ST_COL_NEXT: Advance to next Doppler column or finish
        // ================================================================
        ST_COL_NEXT: begin
            if (col_idx < NUM_DOPPLER_BINS - 1) begin
                col_idx      <= col_idx + 1;
                col_load_idx <= 0;
                mag_raddr    <= {6'd0, col_idx + 5'd1};
                state        <= ST_COL_LOAD;
            end else begin
                state <= ST_DONE;
            end
        end

        // ================================================================
        // ST_DONE: Frame complete, return to idle
        // ================================================================
        ST_DONE: begin
            cfar_status <= 8'd0;
            state <= ST_IDLE;

            `ifdef SIMULATION
            $display("[CFAR] Frame complete: %0d total detections", detect_count);
            `endif
        end

        default: state <= ST_IDLE;
        endcase
    end
end

// ============================================================================
// BRAM + LINE BUFFER INITIALIZATION (simulation only)
// ============================================================================
`ifdef SIMULATION
integer init_i;
initial begin
    for (init_i = 0; init_i < TOTAL_CELLS; init_i = init_i + 1)
        mag_mem[init_i] = 0;
    for (init_i = 0; init_i < NUM_RANGE_BINS; init_i = init_i + 1)
        col_buf[init_i] = 0;
end
`endif

endmodule
