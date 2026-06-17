`timescale 1ns / 1ps

// ============================================================================
// 匹配滤波器模块（matched_filter_multi_segment.v）
// ============================================================================
// 功能概述：
// 本模块使用重叠保留法（Overlap-Save）实现脉冲压缩（匹配滤波）。
// 
// 算法说明：
// 将 3000 样本长啁啾分段为 4 个 1024 点 segment，每个 segment 做 FFT，
// 在频域与参考啁啾相乘，然后 IFFT，去除前 128 点（重叠部分），
// 保留 896 点（SEGMENT_ADVANCE）。
// 
// 处理流程（541 行代码）：
// 1. ST_IDLE：等待新啁啾
// 2. ST_COLLECT_DATA：收集 3000 样本
// 3. ST_ZERO_PAD：零填充到 1024 点
// 4. ST_WAIT_REF：等待参考啁啾数据
// 5. ST_PROCESSING：FFT（使用 xfft_1024 IP）
// 6. ST_WAIT_FFT：等待 FFT 完成
// 7. ST_OUTPUT：输出脉冲压缩结果
// 8. ST_NEXT_SEGMENT：前进到下一个 segment
// 9. ST_OVERLAP_COPY：复制重叠部分（128 点）
// 
// 关键参数：
// - BUFFER_SIZE = 1024（FFT 点数）
// - LONG_CHIRP_SAMPLES = 3000（长啁啾样本数）
// - SHORT_CHIRP_SAMPLES = 50（短啁啾样本数）
// - OVERLAP_SAMPLES = 128（重叠样本数）
// - SEGMENT_ADVANCE = 896（每个 segment 前进样本数）
// 
// 时钟域：clk (100 MHz)
// 输入：ddc_i/q (18bit, 100MHz)
// 输出：pc_i_w/q_w (16bit, 脉冲压缩后)
// ============================================================================

// matched_filter_multi_segment.v
module matched_filter_multi_segment (
    input wire clk,           // 100MHz
    input wire reset_n,
    
    // Input from DDC (100 MSPS)
    input wire signed [17:0] ddc_i,
    input wire signed [17:0] ddc_q,
    input wire ddc_valid,
    
    // Chirp control (from sequence controller)
    input wire use_long_chirp,      // 
    input wire [5:0] chirp_counter, // 
    
    // Microcontroller sync signals
    input wire mc_new_chirp,        // Toggle for new chirp (32)
    input wire mc_new_elevation,    // Toggle for new elevation (32)
    input wire mc_new_azimuth,      // Toggle for new azimuth (50)
    
    input wire [15:0] long_chirp_real,
    input wire [15:0] long_chirp_imag,
    input wire [15:0] short_chirp_real,
    input wire [15:0] short_chirp_imag,
    
    // Memory system interface
    output reg [1:0] segment_request,
    output wire [9:0] sample_addr_out,  // Tell memory which sample we need
    output reg mem_request,
    input wire mem_ready,
    
    // Output: Pulse compressed
    output wire signed [15:0] pc_i_w,
    output wire signed [15:0] pc_q_w,
    output wire pc_valid_w,
    
    // Status
    output reg [3:0] status
);

// ========== FIXED PARAMETERS ==========
parameter BUFFER_SIZE = 1024;
parameter LONG_CHIRP_SAMPLES = 3000;  // Still 3000 samples total
parameter SHORT_CHIRP_SAMPLES = 50;   // 0.5�s @ 100MHz
parameter OVERLAP_SAMPLES = 128;      // Standard for 1024-pt FFT
parameter SEGMENT_ADVANCE = BUFFER_SIZE - OVERLAP_SAMPLES;  // 896 samples
parameter DEBUG = 1;                  // Debug output control

// Calculate segments needed with overlap
// For 3072 samples with 128 overlap: 
// Segments = ceil((3072 - 128) / 896) = ceil(2944/896) = 4
parameter LONG_SEGMENTS = 4;          // Now exactly 4 segments!
parameter SHORT_SEGMENTS = 1;         // 50 samples padded to 1024

// ========== FIXED INTERNAL SIGNALS ==========
reg signed [31:0] pc_i, pc_q;
reg pc_valid;

// Dual buffer for overlap-save — BRAM inferred for synthesis
(* ram_style = "block" *) reg signed [15:0] input_buffer_i [0:BUFFER_SIZE-1];
(* ram_style = "block" *) reg signed [15:0] input_buffer_q [0:BUFFER_SIZE-1];
reg [10:0] buffer_write_ptr;
reg [10:0] buffer_read_ptr;
reg buffer_has_data;
reg buffer_processing;
reg [15:0] chirp_samples_collected;

// BRAM write port signals
reg        buf_we;
reg [9:0]  buf_waddr;
reg signed [15:0] buf_wdata_i, buf_wdata_q;

// BRAM read port signals
reg [9:0]  buf_raddr;
reg signed [15:0] buf_rdata_i, buf_rdata_q;

// State machine
reg [3:0] state;
localparam ST_IDLE = 0;
localparam ST_COLLECT_DATA = 1;
localparam ST_ZERO_PAD = 2;
localparam ST_WAIT_REF = 3;
localparam ST_PROCESSING = 4;
localparam ST_WAIT_FFT = 5;
localparam ST_OUTPUT = 6;
localparam ST_NEXT_SEGMENT = 7;
localparam ST_OVERLAP_COPY = 8;

// Segment tracking
reg [2:0] current_segment;        // 0-3
reg [2:0] total_segments;
reg segment_done;
reg chirp_complete;
reg saw_chain_output;             // Flag: chain started producing output

// Overlap cache — captured during ST_PROCESSING, written back in ST_OVERLAP_COPY
reg signed [15:0] overlap_cache_i [0:OVERLAP_SAMPLES-1];
reg signed [15:0] overlap_cache_q [0:OVERLAP_SAMPLES-1];
reg [7:0] overlap_copy_count;

// Microcontroller sync detection
reg mc_new_chirp_prev, mc_new_elevation_prev, mc_new_azimuth_prev;
wire chirp_start_pulse = mc_new_chirp && !mc_new_chirp_prev;
wire elevation_change_pulse = mc_new_elevation && !mc_new_elevation_prev;
wire azimuth_change_pulse = mc_new_azimuth && !mc_new_azimuth_prev;

// Processing chain signals
wire [15:0] fft_pc_i, fft_pc_q;
wire fft_pc_valid;
wire [3:0] fft_chain_state;

// Buffer for FFT input
reg [15:0] fft_input_i, fft_input_q;
reg fft_input_valid;
reg fft_start;

// ========== SAMPLE ADDRESS OUTPUT ==========
assign sample_addr_out = buffer_read_ptr;

// ========== MICROCONTROLLER SYNC ==========
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        mc_new_chirp_prev <= 1'b0;
        mc_new_elevation_prev <= 1'b0;
        mc_new_azimuth_prev <= 1'b0;
    end else begin
        mc_new_chirp_prev <= mc_new_chirp;
        mc_new_elevation_prev <= mc_new_elevation;
        mc_new_azimuth_prev <= mc_new_azimuth;
    end
end

// ========== BUFFER INITIALIZATION ==========
integer buf_init;
integer ov_init;
initial begin
    for (buf_init = 0; buf_init < BUFFER_SIZE; buf_init = buf_init + 1) begin
        input_buffer_i[buf_init] = 16'd0;
        input_buffer_q[buf_init] = 16'd0;
    end
    for (ov_init = 0; ov_init < OVERLAP_SAMPLES; ov_init = ov_init + 1) begin
        overlap_cache_i[ov_init] = 16'd0;
        overlap_cache_q[ov_init] = 16'd0;
    end
end

// ========== BRAM WRITE PORT (synchronous, no async reset) ==========
always @(posedge clk) begin
    if (buf_we) begin
        input_buffer_i[buf_waddr] <= buf_wdata_i;
        input_buffer_q[buf_waddr] <= buf_wdata_q;
    end
end

// ========== BRAM READ PORT (synchronous, no async reset) ==========
always @(posedge clk) begin
    buf_rdata_i <= input_buffer_i[buf_raddr];
    buf_rdata_q <= input_buffer_q[buf_raddr];
end

// ========== FIXED STATE MACHINE WITH OVERLAP-SAVE ==========
integer i;
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        state <= ST_IDLE;
        buffer_write_ptr <= 0;
        buffer_read_ptr <= 0;
        buffer_has_data <= 0;
        buffer_processing <= 0;
        current_segment <= 0;
        segment_done <= 0;
        segment_request <= 0;
        mem_request <= 0;
        pc_valid <= 0;
        status <= 0;
        chirp_samples_collected <= 0;
        chirp_complete <= 0;
        saw_chain_output <= 0;
        fft_input_valid <= 0;
        fft_start <= 0;
        buf_we <= 0;
        buf_waddr <= 0;
        buf_wdata_i <= 0;
        buf_wdata_q <= 0;
        buf_raddr <= 0;
        overlap_copy_count <= 0;
    end else begin
        pc_valid <= 0;
        mem_request <= 0;
        fft_input_valid <= 0;
        buf_we <= 0;  // Default: no write
        
        case (state)
            ST_IDLE: begin
                // Reset for new chirp
                buffer_write_ptr <= 0;
                buffer_read_ptr <= 0;
                buffer_has_data <= 0;
                buffer_processing <= 0;
                current_segment <= 0;
                segment_done <= 0;
                chirp_samples_collected <= 0;
                chirp_complete <= 0;
                saw_chain_output <= 0;
                
                // Wait for chirp start from microcontroller
                if (chirp_start_pulse) begin
                    state <= ST_COLLECT_DATA;
                    total_segments <= use_long_chirp ? LONG_SEGMENTS[2:0] : SHORT_SEGMENTS[2:0];
                    
                    `ifdef SIMULATION
                    $display("[MULTI_SEG_FIXED] Starting %s chirp, segments: %d",
                             use_long_chirp ? "LONG" : "SHORT", 
                             use_long_chirp ? LONG_SEGMENTS : SHORT_SEGMENTS);
                    $display("[MULTI_SEG_FIXED] Overlap: %d samples, Advance: %d samples",
                             OVERLAP_SAMPLES, SEGMENT_ADVANCE);
                    `endif
                end
            end
            
            ST_COLLECT_DATA: begin
                // Collect samples for current segment with overlap-save
                if (ddc_valid && buffer_write_ptr < BUFFER_SIZE) begin
                    // Store in buffer via BRAM write port
                    buf_we <= 1;
                    buf_waddr <= buffer_write_ptr[9:0];
                    buf_wdata_i <= ddc_i[17:2] + ddc_i[1];
                    buf_wdata_q <= ddc_q[17:2] + ddc_q[1];
                    
                    buffer_write_ptr <= buffer_write_ptr + 1;
                    chirp_samples_collected <= chirp_samples_collected + 1;
                    
                    // Debug: Show first few samples
                    if (chirp_samples_collected < 10 && buffer_write_ptr < 10) begin
                        `ifdef SIMULATION
                        $display("[MULTI_SEG_FIXED] Store[%0d]: I=%h Q=%h", 
                                 buffer_write_ptr, 
                                 ddc_i[17:2] + ddc_i[1], 
                                 ddc_q[17:2] + ddc_q[1]);
                        `endif
                    end
                    
                    // SHORT CHIRP: Only 50 samples, then zero-pad
                    if (!use_long_chirp) begin
                        if (chirp_samples_collected >= SHORT_CHIRP_SAMPLES - 1) begin
                            state <= ST_ZERO_PAD;
                            `ifdef SIMULATION
                            $display("[MULTI_SEG_FIXED] Short chirp: collected %d samples, starting zero-pad",
                                     chirp_samples_collected + 1);
                            `endif
                        end
                    end
                end
                
                // LONG CHIRP: segment-ready and chirp-complete checks
                // evaluated every clock (not gated by ddc_valid) to avoid
                // missing the transition when buffer_write_ptr updates via
                // non-blocking assignment one cycle after the last write.
                //
                // Overlap-save fix: fill the FULL 1024-sample buffer before
                // processing.  For segment 0 this means 1024 fresh samples.
                // For segments 1+, write_ptr starts at OVERLAP_SAMPLES (128)
                // so we collect 896 new samples to fill the buffer.
                if (use_long_chirp) begin
                    if (buffer_write_ptr >= BUFFER_SIZE) begin
                        buffer_has_data <= 1;
                        state <= ST_WAIT_REF;
                        segment_request <= current_segment[1:0];
                        mem_request <= 1;
                        
                        `ifdef SIMULATION
                        $display("[MULTI_SEG_FIXED] Segment %d ready: %d samples collected",
                                 current_segment, chirp_samples_collected);
                        `endif
                    end
                    
                    if (chirp_samples_collected >= LONG_CHIRP_SAMPLES && !chirp_complete) begin
                        chirp_complete <= 1;
                        `ifdef SIMULATION
                        $display("[MULTI_SEG_FIXED] End of long chirp reached");
                        `endif
                        // If buffer isn't full yet, zero-pad the remainder
                        // (last segment with fewer than 896 new samples)
                        if (buffer_write_ptr < BUFFER_SIZE) begin
                            state <= ST_ZERO_PAD;
                            `ifdef SIMULATION
                            $display("[MULTI_SEG_FIXED] Last segment partial: zero-padding from %0d to %0d",
                                     buffer_write_ptr, BUFFER_SIZE - 1);
                            `endif
                        end
                    end
                end
            end
            
            ST_ZERO_PAD: begin
                // Zero-pad remaining buffer via BRAM write port
                buf_we <= 1;
                buf_waddr <= buffer_write_ptr[9:0];
                buf_wdata_i <= 16'd0;
                buf_wdata_q <= 16'd0;
                buffer_write_ptr <= buffer_write_ptr + 1;
                
                if (buffer_write_ptr >= BUFFER_SIZE - 1) begin
                    // Done zero-padding
                    buffer_has_data <= 1;
                    buffer_write_ptr <= 0;
                    state <= ST_WAIT_REF;
                    segment_request <= use_long_chirp ? current_segment[1:0] : 2'd0;
                    mem_request <= 1;
                    `ifdef SIMULATION
                    $display("[MULTI_SEG_FIXED] Zero-pad complete, buffer full");
                    `endif
                end
            end
            
            ST_WAIT_REF: begin
                // Wait for memory to provide reference coefficients
                buf_raddr <= 10'd0;  // Pre-present addr 0 so buf_rdata is ready next cycle
                if (mem_ready) begin
                    // Start processing — buf_rdata[0] will be valid on FIRST clock of ST_PROCESSING
                    buffer_processing <= 1;
                    buffer_read_ptr <= 0;
                    fft_start <= 1;
                    state <= ST_PROCESSING;
                    
                    `ifdef SIMULATION
                    $display("[MULTI_SEG_FIXED] Reference ready, starting processing segment %d",
                             current_segment);
                    `endif
                end
            end
            
            ST_PROCESSING: begin
                // Feed data to FFT chain from BRAM.
                // buf_raddr was pre-presented in ST_WAIT_REF (=0), so
                // buf_rdata already contains data[0] on the first clock here.
                // Each cycle: feed buf_rdata, present NEXT address.
                if ((buffer_processing) && (buffer_read_ptr < BUFFER_SIZE)) begin
                    // 1. Feed BRAM read data to FFT (valid for current buffer_read_ptr)
                    fft_input_i <= buf_rdata_i;
                    fft_input_q <= buf_rdata_q;
                    fft_input_valid <= 1;
                    
                    // 2. Request corresponding reference sample
                    mem_request <= 1'b1;
                    
                    // 3. Cache tail samples for overlap-save
                    if (buffer_read_ptr >= SEGMENT_ADVANCE) begin
                        overlap_cache_i[buffer_read_ptr - SEGMENT_ADVANCE] <= buf_rdata_i;
                        overlap_cache_q[buffer_read_ptr - SEGMENT_ADVANCE] <= buf_rdata_q;
                    end
                    
                    // Debug every 100 samples
                    if (buffer_read_ptr % 100 == 0) begin
                        `ifdef SIMULATION
                        $display("[MULTI_SEG_FIXED] Processing[%0d]: ADC I=%h Q=%h",
                                buffer_read_ptr,
                                buf_rdata_i,
                                buf_rdata_q);
                        `endif
                    end
                    
                    // Present NEXT read address (for next cycle)
                    buf_raddr <= buffer_read_ptr[9:0] + 10'd1;
                    buffer_read_ptr <= buffer_read_ptr + 1;
                    
                end else if (buffer_read_ptr >= BUFFER_SIZE) begin
                    // Done feeding buffer
                    fft_input_valid <= 0;
                    mem_request <= 0;
                    buffer_processing <= 0;
                    buffer_has_data <= 0;
                    saw_chain_output <= 0;
                    state <= ST_WAIT_FFT;  // CRITICAL: Wait for FFT completion
                    
                    `ifdef SIMULATION
                    $display("[MULTI_SEG_FIXED] Finished feeding %d samples to FFT, waiting...",
                             BUFFER_SIZE);
                    `endif
                end
            end
            
            ST_WAIT_FFT: begin
                // Wait for the processing chain to complete ALL outputs.
                // The chain streams 1024 samples (fft_pc_valid=1 for 1024 clocks),
                // then transitions to ST_DONE (9) -> ST_IDLE (0).
                // We track when output starts (saw_chain_output) and only
                // proceed once the chain returns to idle after outputting.
                if (fft_pc_valid) begin
                    saw_chain_output <= 1;
                end
                
                if (saw_chain_output && fft_chain_state == 4'd0) begin
                    // Chain has returned to idle after completing all output
                    saw_chain_output <= 0;
                    state <= ST_OUTPUT;
                    `ifdef SIMULATION
                    $display("[MULTI_SEG_FIXED] Chain complete for segment %d, entering ST_OUTPUT",
                             current_segment);
                    `endif
                end
            end
            
            ST_OUTPUT: begin
                // Store FFT output
                pc_i <= fft_pc_i;
                pc_q <= fft_pc_q;
                pc_valid <= 1;
                segment_done <= 1;
                
                `ifdef SIMULATION
                $display("[MULTI_SEG_FIXED] Output segment %d: I=%h Q=%h",
                         current_segment, fft_pc_i, fft_pc_q);
                `endif
                
                // Check if we need more segments
                if (current_segment < total_segments - 1 || !chirp_complete) begin
                    state <= ST_NEXT_SEGMENT;
                end else begin
                    // All segments complete
                    state <= ST_IDLE;
                    `ifdef SIMULATION
                    $display("[MULTI_SEG_FIXED] All %d segments complete",
                             total_segments);
                    `endif
                end
            end
            
            ST_NEXT_SEGMENT: begin
                // Prepare for next segment with OVERLAP-SAVE
                current_segment <= current_segment + 1;
                segment_done <= 0;
                
                if (use_long_chirp) begin
                    // OVERLAP-SAVE: Write cached tail samples back to BRAM [0..127]
                    overlap_copy_count <= 0;
                    state <= ST_OVERLAP_COPY;
                    
                    `ifdef SIMULATION
                    $display("[MULTI_SEG_FIXED] Overlap-save: writing %d cached samples",
                             OVERLAP_SAMPLES);
                    `endif
                end else begin
                    // Short chirp: only one segment
                    buffer_write_ptr <= 0;
                    if (!chirp_complete) begin
                        state <= ST_COLLECT_DATA;
                    end else begin
                        state <= ST_IDLE;
                    end
                end
            end
            
            ST_OVERLAP_COPY: begin
                // Write one cached overlap sample per cycle to BRAM
                buf_we <= 1;
                buf_waddr <= {{2{1'b0}}, overlap_copy_count};
                buf_wdata_i <= overlap_cache_i[overlap_copy_count];
                buf_wdata_q <= overlap_cache_q[overlap_copy_count];
                
                if (overlap_copy_count < OVERLAP_SAMPLES - 1) begin
                    overlap_copy_count <= overlap_copy_count + 1;
                end else begin
                    // All 128 samples written back
                    buffer_write_ptr <= OVERLAP_SAMPLES;
                    
                    `ifdef SIMULATION
                    $display("[MULTI_SEG_FIXED] Overlap-save: copied %d samples, write_ptr=%d",
                             OVERLAP_SAMPLES, OVERLAP_SAMPLES);
                    `endif
                    
                    if (!chirp_complete) begin
                        state <= ST_COLLECT_DATA;
                    end else begin
                        state <= ST_IDLE;
                    end
                end
            end
            
            default: begin
                state <= ST_IDLE;
            end
        endcase
        
        // Update status
        status <= {state[2:0], use_long_chirp};
    end
end

// ========== PROCESSING CHAIN INSTANTIATION ==========
matched_filter_processing_chain m_f_p_c(
    .clk(clk),
    .reset_n(reset_n),
    
    // Input ADC Data
    .adc_data_i(fft_input_i),
    .adc_data_q(fft_input_q),
    .adc_valid(fft_input_valid),// && buffer_processing),
    
    // Chirp Selection
    .chirp_counter(chirp_counter),
    
    // Reference Chirp Memory Interfaces
    .long_chirp_real(long_chirp_real),
    .long_chirp_imag(long_chirp_imag),
    .short_chirp_real(short_chirp_real),
    .short_chirp_imag(short_chirp_imag),
    
    // Output
    .range_profile_i(fft_pc_i),
    .range_profile_q(fft_pc_q),
    .range_profile_valid(fft_pc_valid),
    
    // Status
    .chain_state(fft_chain_state)
);

// ========== DEBUG MONITOR ==========
`ifdef SIMULATION
reg [31:0] dbg_cycles;
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        dbg_cycles <= 0;
    end else begin
        dbg_cycles <= dbg_cycles + 1;
        
        // Monitor state transitions
        if (dbg_cycles % 1000 == 0 && state != ST_IDLE) begin
            $display("[MULTI_SEG_MONITOR @%0d] state=%0d, segment=%0d/%0d, samples=%0d",
                     dbg_cycles, state, current_segment, total_segments,
                     chirp_samples_collected);
        end
    end
end
`endif

// ========== OUTPUT CONNECTIONS ==========
assign pc_i_w = fft_pc_i;
assign pc_q_w = fft_pc_q;
assign pc_valid_w = fft_pc_valid;

endmodule