`timescale 1ns / 1ps

/**
 * mti_canceller.v
 *
 * Moving Target Indication (MTI) — configurable 2/3-pulse canceller
 * for ground clutter removal.
 *
 * [OPT-IMPROVE] Added 3-pulse canceller option (PULSES=3) for 2× better
 * clutter rejection. The 3-pulse canceller implements H(z) = 1 - 2z^{-1} + z^{-2},
 * providing ~12 dB/Hz steeper notch than 2-pulse at DC, at the cost of
 * one additional BRAM line and 2 extra subtractors.
 *
 * Sits between the range bin decimator and the Doppler processor in the
 * AERIS-10 receiver chain.
 *
 * Parameter PULSES:
 *   2 (default) = 2-pulse: H(z) = 1 - z^{-1}     (6 dB/clutter improvement)
 *   3            = 3-pulse: H(z) = 1 - 2z^{-1} + z^{-2}  (12 dB improvement)
 *
 * Signal chain position:
 *   Range Bin Decimator → [MTI Canceller] → Doppler Processor
 *
 * Algorithm (2-pulse):
 *   mti_out[r] = current[r] - previous[r]
 *
 * Algorithm (3-pulse):
 *   mti_out[r] = current[r] - 2*previous[r] + preprevious[r]
 *
 * The previous chirp's range bins are stored in BRAM.
 * On the very first chirps after reset (or enable), output is zero (muted).
 *
 * When mti_enable=0, the module is a transparent pass-through.
 *
 * Resources:
 *   2-pulse: 2 BRAM18 (64×16 I+Q), ~30 LUTs, ~40 FFs, 0 DSP48
 *   3-pulse: 4 BRAM18 (64×16 I+Q × 2), ~60 LUTs, ~60 FFs, 0 DSP48
 *
 * Clock domain: clk (100 MHz)
 */

module mti_canceller #(
    parameter NUM_RANGE_BINS = 64,
    parameter DATA_WIDTH     = 16,
    parameter PULSES         = 2    // [OPT] 2=two-pulse, 3=three-pulse canceller
) (
    input wire clk,
    input wire reset_n,

    // ========== INPUT (from range bin decimator) ==========
    input wire signed [DATA_WIDTH-1:0] range_i_in,
    input wire signed [DATA_WIDTH-1:0] range_q_in,
    input wire                         range_valid_in,
    input wire [5:0]                   range_bin_in,

    // ========== OUTPUT (to Doppler processor) ==========
    output reg signed [DATA_WIDTH-1:0] range_i_out,
    output reg signed [DATA_WIDTH-1:0] range_q_out,
    output reg                         range_valid_out,
    output reg [5:0]                   range_bin_out,

    // ========== CONFIGURATION ==========
    input wire mti_enable,   // 1=MTI active, 0=pass-through

    // ========== STATUS ==========
    output reg mti_first_chirp  // 1 during first chirp (output muted)
);

// ============================================================================
// PREVIOUS CHIRP BUFFER (64 x 16-bit I, 64 x 16-bit Q)
// [OPT] 3-pulse mode adds preprevious buffer for H(z)=1-2z^{-1}+z^{-2}
// ============================================================================

reg signed [DATA_WIDTH-1:0] prev_i [0:NUM_RANGE_BINS-1];
reg signed [DATA_WIDTH-1:0] prev_q [0:NUM_RANGE_BINS-1];

// [OPT] Preprevious buffer for 3-pulse canceller
reg signed [DATA_WIDTH-1:0] prev2_i [0:NUM_RANGE_BINS-1];
reg signed [DATA_WIDTH-1:0] prev2_q [0:NUM_RANGE_BINS-1];

// Track how many valid chirps we have (0, 1, or 2+)
reg [1:0] valid_chirp_count;

// ============================================================================
// MTI PROCESSING
// ============================================================================

// Read previous chirp data (combinational)
wire signed [DATA_WIDTH-1:0] prev_i_rd = prev_i[range_bin_in];
wire signed [DATA_WIDTH-1:0] prev_q_rd = prev_q[range_bin_in];

// [OPT] Read preprevious chirp data for 3-pulse canceller
wire signed [DATA_WIDTH-1:0] prev2_i_rd = prev2_i[range_bin_in];
wire signed [DATA_WIDTH-1:0] prev2_q_rd = prev2_q[range_bin_in];

// Compute difference with saturation
// 2-pulse: diff = current - previous
// 3-pulse: diff = current - 2*previous + preprevious
wire signed [DATA_WIDTH:0] diff_i_full = {range_i_in[DATA_WIDTH-1], range_i_in}
                                        - {prev_i_rd[DATA_WIDTH-1], prev_i_rd};
wire signed [DATA_WIDTH:0] diff_q_full = {range_q_in[DATA_WIDTH-1], range_q_in}
                                        - {prev_q_rd[DATA_WIDTH-1], prev_q_rd};

// [OPT] 3-pulse: diff3 = current - 2*previous + preprevious
// = (current - previous) + (preprevious - previous)
// = diff2 + (preprevious - previous)
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
// MAIN LOGIC
// [OPT] Updated to support 2-pulse and 3-pulse cancellation modes
// ============================================================================
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        range_i_out     <= {DATA_WIDTH{1'b0}};
        range_q_out     <= {DATA_WIDTH{1'b0}};
        range_valid_out <= 1'b0;
        range_bin_out   <= 6'd0;
        valid_chirp_count <= 2'd0;
        mti_first_chirp <= 1'b1;
    end else begin
        // Default: no valid output
        range_valid_out <= 1'b0;

        if (range_valid_in) begin
            // Always store current sample as "previous" for next chirp
            // [OPT] For 3-pulse: shift prev→prev2, then store current→prev
            if (PULSES == 3) begin
                prev2_i[range_bin_in] <= prev_i[range_bin_in];
                prev2_q[range_bin_in] <= prev_q[range_bin_in];
            end
            prev_i[range_bin_in] <= range_i_in;
            prev_q[range_bin_in] <= range_q_in;

            // Output path
            range_bin_out <= range_bin_in;

            if (!mti_enable) begin
                // Pass-through mode: no MTI processing
                range_i_out     <= range_i_in;
                range_q_out     <= range_q_in;
                range_valid_out <= 1'b1;
                // Reset state when MTI is disabled
                valid_chirp_count <= 2'd0;
                mti_first_chirp <= 1'b1;
            end else if (valid_chirp_count < (PULSES - 1)) begin
                // Not enough history yet: mute output
                // For 2-pulse: need 1 previous chirp (count reaches 1)
                // For 3-pulse: need 2 previous chirps (count reaches 2)
                range_i_out     <= {DATA_WIDTH{1'b0}};
                range_q_out     <= {DATA_WIDTH{1'b0}};
                range_valid_out <= 1'b1;

                // After last range bin of this chirp, increment count
                if (range_bin_in == NUM_RANGE_BINS - 1) begin
                    valid_chirp_count <= valid_chirp_count + 1;
                    if (valid_chirp_count == PULSES - 2) begin
                        mti_first_chirp <= 1'b0;
                    end
                end
            end else begin
                // Normal MTI: subtract previous from current
                range_i_out     <= diff_i_sat;
                range_q_out     <= diff_q_sat;
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
