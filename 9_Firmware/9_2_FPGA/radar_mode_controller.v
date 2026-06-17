`timescale 1ns / 1ps

/**
 * radar_mode_controller.v — 雷达模式控制器
 *
 * 【中文功能概述】
 * 为 AERIS-10 接收机处理链生成波束扫描和啁啾模式控制信号。
 * 本模块驱动以下信号：
 *   - use_long_chirp   : 选择长啁啾（30us）或短啁啾（0.5us）模式
 *   - mc_new_chirp     : 新啁啍开始的翻转信号
 *   - mc_new_elevation : 新俯仰角步进的翻转信号
 *   - mc_new_azimuth   : 新方位角步进的翻转信号
 *
 * 这些信号被 matched_filter_multi_segment 和 chirp_memory_loader_param 消费。
 *
 * 本控制器的扫描序列镜像发射机 plfm_chirp_controller_enhanced 的定义：
 *   - 每个俯仰角 32 个啁啾
 *   - 每个方位角 31 个俯仰角
 *   - 每次完整扫描 50 个方位角
 *   - 每个啁啾：长啁啾 → 听取 → 保护间隔 → 短啁啾 → 听取
 *
 * 【工作模式（mode[1:0]）】
 *   2'b00 = STM32 驱动模式（直通 STM32 翻转信号）
 *   2'b01 = 自动扫描模式（内部时序自动运行）
 *   2'b10 = 单啁啍调试模式（每次触发发射一个啁啾）
 *   2'b11 = 保留
 *
 * 【状态机】S_IDLE → S_LONG_CHIRP → S_LONG_LISTEN → S_GUARD → S_SHORT_CHIRP → S_SHORT_LISTEN → S_ADVANCE
 *
 * 【时序参数（100MHz 时钟周期）】
 *   长啁啾：30us = 3000 周期    |  短啁啾：0.5us = 50 周期
 *   长听取：137us = 13700 周期  |  短听取：174.5us = 17450 周期
 *   保护间隔：175.4us = 17540 周期
 *
 * Clock domain: clk (100 MHz)
 */

module radar_mode_controller #(
    // ========== 扫描参数（Scan Parameters） ==========
    parameter CHIRPS_PER_ELEVATION = 32,     // 每个俯仰角的啁啾数
    parameter ELEVATIONS_PER_AZIMUTH = 31,   // 每个方位角的俯仰角数
    parameter AZIMUTHS_PER_SCAN = 50,         // 每次完整扫描的方位角数

    // ========== 时序参数（100MHz 时钟周期）==========
    // 长啁啾：30us = 3000 周期 | 长听取：137us = 13700 周期
    // 保护间隔：175.4us = 17540 周期
    // 短啁啾：0.5us = 50 周期   | 短听取：174.5us = 17450 周期
    parameter LONG_CHIRP_CYCLES   = 3000,
    parameter LONG_LISTEN_CYCLES  = 13700,
    parameter GUARD_CYCLES        = 17540,
    parameter SHORT_CHIRP_CYCLES  = 50,
    parameter SHORT_LISTEN_CYCLES = 17450
) (
    input wire clk,                    // 系统时钟 100MHz
    input wire reset_n,                // 异步复位（低有效）

    // ========== 模式选择（Mode Selection） ==========
    input wire [1:0] mode,            // 00=STM32驱动, 01=自动扫描, 10=单啁啍调试, 11=保留

    // ========== STM32 直通输入（mode=00 时有效）==========
    input wire stm32_new_chirp,       // STM32 啁啾触发翻转信号
    input wire stm32_new_elevation,   // STM32 俯仰角触发翻转信号
    input wire stm32_new_azimuth,     // STM32 方位角触发翻转信号

    // ========== 单啁啍触发（mode=10 时有效）==========
    input wire trigger,               // 外部触发脉冲（每发一次射一个啁啾）

    // ========== 运行时可配置时序（Gap 2：主机 USB 命令配置）==========
    // 当连接时，这些输入覆盖编译时默认参数值。
    // 当保持默认值（实例化时绑定为参数值），行为与修改前完全一致。
    input wire [15:0] cfg_long_chirp_cycles,      // 长啁啾周期数（操作码 0x10）
    input wire [15:0] cfg_long_listen_cycles,     // 长听取周期数（操作码 0x11）
    input wire [15:0] cfg_guard_cycles,           // 保护间隔周期数（操作码 0x12）
    input wire [15:0] cfg_short_chirp_cycles,     // 短啁啾周期数（操作码 0x13）
    input wire [15:0] cfg_short_listen_cycles,    // 短听取周期数（操作码 0x14）
    input wire [5:0]  cfg_chirps_per_elev,        // 每俯仰角啁啍数（操作码 0x15）

    // ========== 输出至接收机处理链（Output to Receiver Chain） ==========
    output reg use_long_chirp,        // 1=长啁啍模式, 0=短啁啍模式
    output reg mc_new_chirp,          // 新啁啍翻转信号（至匹配滤波器）
    output reg mc_new_elevation,      // 新俯仰角翻转信号
    output reg mc_new_azimuth,        // 新方位角翻转信号

    // ========== 波束位置跟踪（Beam Position Tracking） ==========
    output reg [5:0] chirp_count,         // 当前啁啍计数（0~31）
    output reg [5:0] elevation_count,     // 当前俯仰角计数（0~30）
    output reg [5:0] azimuth_count,       // 当前方位角计数（0~49）

    // ========== 状态输出（Status Outputs） ==========
    output wire scanning,             // 1=扫描进行中
    output wire scan_complete         // 完整扫描完成脉冲

`ifdef FORMAL
    ,
    output wire [2:0]  fv_scan_state,
    output wire [17:0] fv_timer
`endif
);

// ============================================================================
// 内部状态定义（INTERNAL STATE）
// ============================================================================

// ========== 自动扫描状态机（Auto-scan State Machine） ==========
// 状态转移：IDLE → 长啁啾 → 长听取 → 保护 → 短啁啾 → 短听取 → 推进(ADVANCE)
reg [2:0] scan_state;
localparam S_IDLE        = 3'd0;   // 空闲
localparam S_LONG_CHIRP  = 3'd1;   // 发射长啁啾（30us）
localparam S_LONG_LISTEN = 3'd2;   // 长距离听取窗口（137us）
localparam S_GUARD       = 3'd3;   // 保护间隔（收发切换+稳定）
localparam S_SHORT_CHIRP = 3'd4;   // 发射短啁啾（0.5us）
localparam S_SHORT_LISTEN = 3'd5;  // 短距离听取窗口（174.5us）
localparam S_ADVANCE     = 3'd6;   // 推进到下一个啁啍/俯仰/方位

// ========== 时序计数器（Timing Counter） ==========
reg [17:0] timer;  // 最大 262143 周期（@100MHz 约 2.6ms，足够所有时序段）

`ifdef FORMAL
assign fv_scan_state = scan_state;
assign fv_timer      = timer;
`endif

// Edge detection for STM32 pass-through
reg stm32_new_chirp_prev;
reg stm32_new_elevation_prev;
reg stm32_new_azimuth_prev;

// Trigger edge detection (for single-chirp mode)
reg trigger_prev;
wire trigger_pulse = trigger & ~trigger_prev;

// Scan completion
reg scan_done_pulse;

// ============================================================================
// STM32 直通模式边沿检测（Edge Detection for STM32 Pass-through）
// ============================================================================
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        stm32_new_chirp_prev     <= 1'b0;
        stm32_new_elevation_prev <= 1'b0;
        stm32_new_azimuth_prev   <= 1'b0;
        trigger_prev             <= 1'b0;
    end else begin
        // 保存上一周期的值，用于 XOR 边沿检测
        stm32_new_chirp_prev     <= stm32_new_chirp;
        stm32_new_elevation_prev <= stm32_new_elevation;
        stm32_new_azimuth_prev   <= stm32_new_azimuth;
        trigger_prev             <= trigger;
    end
end
// 翻转检测：当前值 ⊕ 上一周期值 = 边沿脉冲
wire stm32_chirp_toggle     = stm32_new_chirp     ^ stm32_new_chirp_prev;
wire stm32_elevation_toggle = stm32_new_elevation  ^ stm32_new_elevation_prev;
wire stm32_azimuth_toggle   = stm32_new_azimuth    ^ stm32_new_azimuth_prev;

// ============================================================================
// 【核心】主状态机（MAIN STATE MACHINE）
// ============================================================================
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        // 复位：所有状态归零
        scan_state      <= S_IDLE;
        timer           <= 18'd0;
        use_long_chirp  <= 1'b1;         // 默认长啁啍模式
        mc_new_chirp    <= 1'b0;
        mc_new_elevation <= 1'b0;
        mc_new_azimuth  <= 1'b0;
        chirp_count     <= 6'd0;
        elevation_count <= 6'd0;
        azimuth_count   <= 6'd0;
        scan_done_pulse <= 1'b0;
    end else begin
        // 清除一次性脉冲信号
        scan_done_pulse <= 1'b0;

        case (mode)
        // ================================================================
        // 模式 00：STM32 驱动直通（STM32-driven pass-through）
        // STM32 固件控制时序；我们只检测翻转边沿并转发到接收链。
        // ================================================================
        2'b00: begin
            // 重置自动扫描状态（本模式下不使用）
            scan_state <= S_IDLE;
            timer      <= 18'd0;

            // 直通翻转信号：STM32 有边沿我们就转发
            if (stm32_chirp_toggle) begin
                mc_new_chirp <= ~mc_new_chirp;   // 翻转输出信号
                use_long_chirp <= 1'b1;          // 默认长啁啾

                // 跟踪啁啍计数（Gap 2: 使用运行时 cfg_chirps_per_elev）
                if (chirp_count < cfg_chirps_per_elev - 1)
                    chirp_count <= chirp_count + 1;
                else
                    chirp_count <= 6'd0;          // 一个俯仰角完成，归零
            end

            if (stm32_elevation_toggle) begin
                mc_new_elevation <= ~mc_new_elevation;
                chirp_count <= 6'd0;              // 啁啍计数归零

                if (elevation_count < ELEVATIONS_PER_AZIMUTH - 1)
                    elevation_count <= elevation_count + 1;
                else
                    elevation_count <= 6'd0;      // 一个方位角完成，归零
            end

            if (stm32_azimuth_toggle) begin
                mc_new_azimuth <= ~mc_new_azimuth;
                elevation_count <= 6'd0;

                if (azimuth_count < AZIMUTHS_PER_SCAN - 1)
                    azimuth_count <= azimuth_count + 1;
                else begin
                    azimuth_count <= 6'd0;         // 完整扫描完成！
                    scan_done_pulse <= 1'b1;       // 发出扫描完成脉冲
                end
            end
        end

        // ================================================================
        // 模式 01：自由运行自动扫描（Free-running auto-scan）
        // 内部生成与发射机匹配的啁啍时序，无需 STM32 参与。
        // ================================================================
        2'b01: begin
            case (scan_state)
            S_IDLE: begin
                // Start first chirp immediately
                scan_state     <= S_LONG_CHIRP;
                timer          <= 18'd0;
                use_long_chirp <= 1'b1;
                mc_new_chirp   <= ~mc_new_chirp;  // Toggle to start chirp
                chirp_count    <= 6'd0;
                elevation_count <= 6'd0;
                azimuth_count  <= 6'd0;

                `ifdef SIMULATION
                $display("[MODE_CTRL] Auto-scan starting");
                `endif
            end

            S_LONG_CHIRP: begin
                use_long_chirp <= 1'b1;
                if (timer < cfg_long_chirp_cycles - 1)
                    timer <= timer + 1;
                else begin
                    timer <= 18'd0;
                    scan_state <= S_LONG_LISTEN;
                end
            end

            S_LONG_LISTEN: begin
                if (timer < cfg_long_listen_cycles - 1)
                    timer <= timer + 1;
                else begin
                    timer <= 18'd0;
                    scan_state <= S_GUARD;
                end
            end

            S_GUARD: begin
                if (timer < cfg_guard_cycles - 1)
                    timer <= timer + 1;
                else begin
                    timer <= 18'd0;
                    scan_state <= S_SHORT_CHIRP;
                    use_long_chirp <= 1'b0;
                end
            end

            S_SHORT_CHIRP: begin
                use_long_chirp <= 1'b0;
                if (timer < cfg_short_chirp_cycles - 1)
                    timer <= timer + 1;
                else begin
                    timer <= 18'd0;
                    scan_state <= S_SHORT_LISTEN;
                end
            end

            S_SHORT_LISTEN: begin
                if (timer < cfg_short_listen_cycles - 1)
                    timer <= timer + 1;
                else begin
                    timer <= 18'd0;
                    scan_state <= S_ADVANCE;
                end
            end

            S_ADVANCE: begin
                // Advance chirp/elevation/azimuth counters
                // (Gap 2: use runtime cfg_chirps_per_elev)
                if (chirp_count < cfg_chirps_per_elev - 1) begin
                    // Next chirp in current elevation
                    chirp_count  <= chirp_count + 1;
                    mc_new_chirp <= ~mc_new_chirp;
                    scan_state   <= S_LONG_CHIRP;
                    use_long_chirp <= 1'b1;
                end else begin
                    chirp_count <= 6'd0;

                    if (elevation_count < ELEVATIONS_PER_AZIMUTH - 1) begin
                        // Next elevation
                        elevation_count  <= elevation_count + 1;
                        mc_new_chirp     <= ~mc_new_chirp;
                        mc_new_elevation <= ~mc_new_elevation;
                        scan_state       <= S_LONG_CHIRP;
                        use_long_chirp   <= 1'b1;
                    end else begin
                        elevation_count <= 6'd0;

                        if (azimuth_count < AZIMUTHS_PER_SCAN - 1) begin
                            // Next azimuth
                            azimuth_count    <= azimuth_count + 1;
                            mc_new_chirp     <= ~mc_new_chirp;
                            mc_new_elevation <= ~mc_new_elevation;
                            mc_new_azimuth   <= ~mc_new_azimuth;
                            scan_state       <= S_LONG_CHIRP;
                            use_long_chirp   <= 1'b1;
                        end else begin
                            // Full scan complete — restart
                            azimuth_count   <= 6'd0;
                            scan_done_pulse <= 1'b1;
                            mc_new_chirp    <= ~mc_new_chirp;
                            mc_new_elevation <= ~mc_new_elevation;
                            mc_new_azimuth  <= ~mc_new_azimuth;
                            scan_state      <= S_LONG_CHIRP;
                            use_long_chirp  <= 1'b1;

                            `ifdef SIMULATION
                            $display("[MODE_CTRL] Full scan complete, restarting");
                            `endif
                        end
                    end
                end
            end

            default: scan_state <= S_IDLE;
            endcase
        end

        // ================================================================
        // 模式 10：单啁啍调试模式（Single-chirp debug mode）
        // 每次触发脉冲发射一个长啁啍，不进行扫描。
        // ================================================================
        2'b10: begin
            case (scan_state)
            S_IDLE: begin
                if (trigger_pulse) begin
                    scan_state     <= S_LONG_CHIRP;
                    timer          <= 18'd0;
                    use_long_chirp <= 1'b1;
                    mc_new_chirp   <= ~mc_new_chirp;
                end
            end

            S_LONG_CHIRP: begin
                if (timer < cfg_long_chirp_cycles - 1)
                    timer <= timer + 1;
                else begin
                    timer <= 18'd0;
                    scan_state <= S_LONG_LISTEN;
                end
            end

            S_LONG_LISTEN: begin
                if (timer < cfg_long_listen_cycles - 1)
                    timer <= timer + 1;
                else begin
                    // Single chirp done, return to idle
                    timer      <= 18'd0;
                    scan_state <= S_IDLE;
                end
            end

            default: scan_state <= S_IDLE;
            endcase
        end

        // ================================================================
        // 模式 11：保留（Reserved — 空闲）
        // ================================================================
        2'b11: begin
            scan_state <= S_IDLE;
            timer      <= 18'd0;
        end

        endcase
    end
end

// ============================================================================
// 输出赋值（OUTPUT ASSIGNMENTS）
// ============================================================================
assign scanning      = (scan_state != S_IDLE);   // 状态非空闲 = 扫描中
assign scan_complete = scan_done_pulse;            // 完整扫描完成脉冲

endmodule
