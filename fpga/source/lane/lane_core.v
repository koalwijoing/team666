// ============================================================
// lane_core.v - lane detection core after edge extraction
// ============================================================
// 算法来源: ai/scripts/43_车道线检测_逐行重映射.py full pipeline
// 参数来源: 21 rows, COV 4/9/14, GAP_DASH=2, N_MIN=8
// v2: width-aware candidates, wide-run exclusion, geometry/mirror gating.
// 验收判据: tb_lane_core.v + tb_golden_l1.v real-frame regression
// Accepts a binary edge stream, captures selected scan rows, then processes
// the frame row by row through remap, run extraction, tracking, fitting,
// solid/dashed classification, and 5-frame voting.
// ============================================================
`timescale 1ns/1ps

module lane_core #(
    parameter integer WIDTH       = 1280,
    parameter integer X_WIDTH     = 11,
    parameter integer Y_WIDTH     = 11,
    parameter integer SCAN_ROWS   = 21,
    parameter integer SCAN_Y0     = 432,
    parameter integer SCAN_STEP   = 12,
    parameter integer MIN_RUN     = 3,
    parameter integer MAX_RUN     = 28,
    parameter integer N_MIN       = 8,
    parameter integer HOLD_MAX    = 8,
    parameter integer MIN_POINTS  = 2,
    parameter integer COV_NONE     = 4,
    parameter integer COV_OCCLUDED = 9,
    parameter integer COV_SOLID    = 14,
    parameter integer GAP_DASH     = 2,
    parameter integer MAX_RESIDUAL = 56,
    parameter integer IDEAL_WIDTH  = 8
) (
    input  wire                 clk,
    input  wire                 rst_n,

    input  wire                 frame_start_i,
    input  wire                 edge_valid_i,
    input  wire                 edge_bit_i,
    input  wire [X_WIDTH-1:0]   x_i,
    input  wire [Y_WIDTH-1:0]   y_i,
    input  wire                 eol_i,
    input  wire                 frame_done_i,

    input  wire                 coeff_wr_i,
    input  wire [4:0]           coeff_row_i,
    input  wire [31:0]          coeff_start_q16_i,
    input  wire [31:0]          coeff_step_q16_i,

    input  wire [10:0]          expected_left_i,
    input  wire [10:0]          expected_right_i,
    input  wire [10:0]          window_i,

    output wire [1:0]           left_state_o,
    output wire [1:0]           right_state_o,
    output wire signed [31:0]   left_a_q8_o,
    output wire signed [31:0]   right_a_q8_o,
    output wire signed [31:0]   left_b_q8_o,
    output wire signed [31:0]   right_b_q8_o,
    output wire                 left_fit_valid_o,
    output wire                 right_fit_valid_o,
    output wire [5:0]           debug_left_point_count_o,
    output wire [5:0]           debug_right_point_count_o,
    output wire                 debug_left_fit_valid_raw_o,
    output wire                 debug_right_fit_valid_raw_o,
    output wire [11:0]          debug_left_max_error_o,
    output wire [11:0]          debug_right_max_error_o,
    output wire [15:0]          dbg_edge_pulses_o,
    output wire [15:0]          dbg_run_events_o,
    output wire                 result_valid_o,
    output wire                 busy_o
);

    localparam [2:0] S_IDLE      = 3'd0;
    localparam [2:0] S_INIT      = 3'd1;
    localparam [2:0] S_ROW_START = 3'd2;
    localparam [2:0] S_REMAP     = 3'd3;
    localparam [2:0] S_ROW_HIT   = 3'd4;
    localparam [2:0] S_END_PULSE = 3'd5;
    localparam [2:0] S_WAIT_FIT  = 3'd6;
    localparam [2:0] S_PUBLISH   = 3'd7;

    reg [2:0] state;
    reg [4:0] proc_row;
    reg [4:0] scan_row_sel;
    reg       remap_start;
    reg       run_row_start;
    reg       track_row_start;
    reg       track_frame_start;
    reg       track_frame_end;
    reg       fit_frame_start;
    reg       fit_frame_end;
    reg       gap_frame_done;
    reg       fit_done_l_seen;
    reg       fit_done_r_seen;

    reg [31:0] coeff_start [0:SCAN_ROWS-1];
    reg [31:0] coeff_step  [0:SCAN_ROWS-1];

    reg        left_hits [0:SCAN_ROWS-1];
    reg        right_hits [0:SCAN_ROWS-1];

    wire [X_WIDTH-1:0] capture_x = x_i;
    wire [Y_WIDTH-1:0] scan_y_end = SCAN_Y0 + SCAN_ROWS * SCAN_STEP - 1;
    wire [Y_WIDTH-1:0] y_delta = y_i - SCAN_Y0;
    wire scan_row_strobe =
        edge_valid_i &&
        (y_i >= SCAN_Y0) &&
        (y_i <= scan_y_end) &&
        ((y_delta % SCAN_STEP) == 0);
    wire [4:0] scan_row_index = y_delta / SCAN_STEP;

    wire capture_rd_bit;
    wire [X_WIDTH-1:0] src_x_count;
    wire src_valid;
    wire remap_src_pop;
    wire remap_dst_valid;
    wire remap_dst_bit;
    wire remap_dst_last;
    wire remap_busy;

    wire run_valid;
    wire [10:0] run_center;
    wire [7:0] run_len;
    wire [4:0] run_row_idx;
    wire [7:0] run_count;
    wire run_row_done;

    wire left_row_hit_valid;
    wire right_row_hit_valid;
    wire left_row_hit;
    wire right_row_hit;
    wire [10:0] left_chosen_x;
    wire [10:0] right_chosen_x;
    wire [5:0] left_track_hits;
    wire [5:0] right_track_hits;
    wire [5:0] left_track_rows;
    wire [5:0] right_track_rows;
    wire [1:0] left_track_state;
    wire [1:0] right_track_state;
    wire [3:0] left_hold_count;
    wire [3:0] right_hold_count;
    wire left_reacquire;
    wire right_reacquire;

    wire left_fit_done;
    wire right_fit_done;
    wire left_fit_valid_raw;
    wire right_fit_valid_raw;
    wire [11:0] left_max_error;
    wire [11:0] right_max_error;
    wire left_fit_busy;
    wire right_fit_busy;
    wire [5:0] left_point_count;
    wire [5:0] right_point_count;

    wire left_gap_result_valid;
    wire right_gap_result_valid;
    wire [1:0] left_gap_state;
    wire [1:0] right_gap_state;
    wire [5:0] left_gap_cov;
    wire [5:0] right_gap_cov;
    wire [5:0] left_gap_gaps;
    wire [5:0] right_gap_gaps;
    wire [5:0] left_gap_rows;
    wire [5:0] right_gap_rows;

    wire left_geom_ok;
    wire right_geom_ok;
    wire geom_done;
    reg  left_geom_ok_r;
    reg  right_geom_ok_r;
    reg  geometry_start;
    reg  geometry_started;
    reg  [1:0] publish_wait;
    wire [1:0] left_voted_state;
    wire [1:0] right_voted_state;

    reg [X_WIDTH-1:0] src_x;

    assign src_x_count = src_x;
    assign src_valid = (src_x < WIDTH);

    scan_row_capture #(
        .WIDTH(WIDTH),
        .X_WIDTH(X_WIDTH),
        .SCAN_ROWS(SCAN_ROWS)
    ) u_capture (
        .clk(clk),
        .rst_n(rst_n),
        .capture_valid_i(scan_row_strobe),
        .capture_row_i(scan_row_index),
        .capture_x_i(capture_x),
        .capture_bit_i(edge_bit_i),
        .rd_row_i(proc_row),
        .rd_x_i(src_x),
        .rd_bit_o(capture_rd_bit)
    );

    row_remap #(
        .WIDTH(WIDTH),
        .X_WIDTH(X_WIDTH)
    ) u_remap (
        .clk(clk),
        .rst_n(rst_n),
        .start_i(remap_start),
        .src_start_q16_i(coeff_start[proc_row]),
        .src_step_q16_i(coeff_step[proc_row]),
        .src_pop_o(remap_src_pop),
        .src_valid_i(src_valid),
        .src_bit_i(capture_rd_bit),
        .dst_valid_o(remap_dst_valid),
        .dst_bit_o(remap_dst_bit),
        .dst_last_o(remap_dst_last),
        .busy_o(remap_busy)
    );

    lane_run_filter #(
        .WIDTH(WIDTH),
        .X_WIDTH(X_WIDTH),
        .MIN_RUN(MIN_RUN),
        .MAX_RUN(MAX_RUN),
        .WIDE_MIN(30),
        .GUARD(24)
    ) u_runs (
        .clk(clk),
        .rst_n(rst_n),
        .row_start_i(run_row_start),
        .row_idx_i(proc_row),
        .bit_valid_i(remap_dst_valid),
        .bit_i(remap_dst_bit),
        .row_end_i(remap_dst_last),
        .run_valid_o(run_valid),
        .run_center_o(run_center),
        .run_len_o(run_len),
        .run_row_idx_o(run_row_idx),
        .run_count_o(run_count),
        .row_done_o(run_row_done)
    );

    lane_track #(
        .N_MIN(N_MIN),
        .HOLD_MAX(HOLD_MAX),
        .IDEAL_WIDTH(IDEAL_WIDTH)
    ) u_track_left (
        .clk(clk),
        .rst_n(rst_n),
        .frame_start_i(track_frame_start),
        .row_start_i(track_row_start),
        .row_idx_i(proc_row),
        .cand_valid_i(run_valid),
        .cand_x_i(run_center),
        .cand_width_i(run_len),
        .row_end_i(run_row_done),
        .frame_end_i(track_frame_end),
        .fit_valid_i(left_fit_valid_raw),
        .fit_a_q8_i(left_a_q8_o),
        .fit_b_q8_i(left_b_q8_o),
        .default_x_i(expected_left_i),
        .window_i(window_i),
        .row_hit_o(left_row_hit),
        .row_hit_valid_o(left_row_hit_valid),
        .chosen_x_o(left_chosen_x),
        .hit_count_o(left_track_hits),
        .row_count_o(left_track_rows),
        .state_o(left_track_state),
        .hold_count_o(left_hold_count),
        .frame_result_valid_o(),
        .reacquire_o(left_reacquire)
    );

    lane_track #(
        .N_MIN(N_MIN),
        .HOLD_MAX(HOLD_MAX),
        .IDEAL_WIDTH(IDEAL_WIDTH)
    ) u_track_right (
        .clk(clk),
        .rst_n(rst_n),
        .frame_start_i(track_frame_start),
        .row_start_i(track_row_start),
        .row_idx_i(proc_row),
        .cand_valid_i(run_valid),
        .cand_x_i(run_center),
        .cand_width_i(run_len),
        .row_end_i(run_row_done),
        .frame_end_i(track_frame_end),
        .fit_valid_i(right_fit_valid_raw),
        .fit_a_q8_i(right_a_q8_o),
        .fit_b_q8_i(right_b_q8_o),
        .default_x_i(expected_right_i),
        .window_i(window_i),
        .row_hit_o(right_row_hit),
        .row_hit_valid_o(right_row_hit_valid),
        .chosen_x_o(right_chosen_x),
        .hit_count_o(right_track_hits),
        .row_count_o(right_track_rows),
        .state_o(right_track_state),
        .hold_count_o(right_hold_count),
        .frame_result_valid_o(),
        .reacquire_o(right_reacquire)
    );

    line_fit #(
        .MIN_POINTS(MIN_POINTS),
        .MAX_POINTS(SCAN_ROWS)
    ) u_fit_left (
        .clk(clk),
        .rst_n(rst_n),
        .frame_start_i(fit_frame_start),
        .point_valid_i(left_row_hit_valid && left_row_hit),
        .row_idx_i(proc_row),
        .x_i(left_chosen_x),
        .frame_end_i(fit_frame_end),
        .point_count_o(left_point_count),
        .a_q8_o(left_a_q8_o),
        .b_q8_o(left_b_q8_o),
        .max_error_o(left_max_error),
        .fit_valid_o(left_fit_valid_raw),
        .busy_o(left_fit_busy),
        .done_o(left_fit_done)
    );

    line_fit #(
        .MIN_POINTS(MIN_POINTS),
        .MAX_POINTS(SCAN_ROWS)
    ) u_fit_right (
        .clk(clk),
        .rst_n(rst_n),
        .frame_start_i(fit_frame_start),
        .point_valid_i(right_row_hit_valid && right_row_hit),
        .row_idx_i(proc_row),
        .x_i(right_chosen_x),
        .frame_end_i(fit_frame_end),
        .point_count_o(right_point_count),
        .a_q8_o(right_a_q8_o),
        .b_q8_o(right_b_q8_o),
        .max_error_o(right_max_error),
        .fit_valid_o(right_fit_valid_raw),
        .busy_o(right_fit_busy),
        .done_o(right_fit_done)
    );

    gap_stat #(
        .TOTAL_ROWS(SCAN_ROWS),
        .COV_NONE(COV_NONE),
        .COV_OCCLUDED(COV_OCCLUDED),
        .COV_SOLID(COV_SOLID),
        .GAP_DASH(GAP_DASH)
    ) u_gap_left (
        .clk(clk),
        .rst_n(rst_n),
        .valid_i(left_row_hit_valid),
        .hit_i(left_row_hit),
        .frame_done(gap_frame_done),
        .state_o(left_gap_state),
        .cov_o(left_gap_cov),
        .gaps_o(left_gap_gaps),
        .row_cnt_o(left_gap_rows),
        .result_valid(left_gap_result_valid)
    );

    gap_stat #(
        .TOTAL_ROWS(SCAN_ROWS),
        .COV_NONE(COV_NONE),
        .COV_OCCLUDED(COV_OCCLUDED),
        .COV_SOLID(COV_SOLID),
        .GAP_DASH(GAP_DASH)
    ) u_gap_right (
        .clk(clk),
        .rst_n(rst_n),
        .valid_i(right_row_hit_valid),
        .hit_i(right_row_hit),
        .frame_done(gap_frame_done),
        .state_o(right_gap_state),
        .cov_o(right_gap_cov),
        .gaps_o(right_gap_gaps),
        .row_cnt_o(right_gap_rows),
        .result_valid(right_gap_result_valid)
    );

    lane_geometry #(
        .WIDTH(WIDTH),
        .SCAN_ROWS(SCAN_ROWS),
        .MAX_RESIDUAL(MAX_RESIDUAL)
    ) u_geometry (
        .clk(clk),
        .rst_n(rst_n),
        .start_i(geometry_start),
        .left_fit_valid_i(left_fit_valid_raw),
        .left_a_q8_i(left_a_q8_o),
        .left_b_q8_i(left_b_q8_o),
        .left_max_error_i(left_max_error),
        .right_fit_valid_i(right_fit_valid_raw),
        .right_a_q8_i(right_a_q8_o),
        .right_b_q8_i(right_b_q8_o),
        .right_max_error_i(right_max_error),
        .left_ok_o(left_geom_ok),
        .right_ok_o(right_geom_ok),
        .done_o(geom_done)
    );

    vote5 u_vote_left (
        .clk(clk),
        .rst_n(rst_n),
        .valid_i(left_gap_result_valid && left_geom_ok_r),
        .state_i(left_gap_state),
        .state_o(left_voted_state)
    );

    vote5 u_vote_right (
        .clk(clk),
        .rst_n(rst_n),
        .valid_i(right_gap_result_valid && right_geom_ok_r),
        .state_i(right_gap_state),
        .state_o(right_voted_state)
    );

    // lane_track state encoding: 0=NO_LINE, 1=LOCKED, 2=HOLD. A locked line
    // survives at most HOLD_MAX frames of low hits and then decays to
    // NO_LINE. Without gating the publish path on that state, the downstream
    // hold chain keeps the last voted state and fit alive indefinitely, so a
    // scene with no lane content still publishes the old line as fresh.
    localparam [1:0] TRACK_STATE_NO_LINE = 2'd0;
    wire left_publish_ok  = (left_track_state  != TRACK_STATE_NO_LINE) && left_geom_ok_r;
    wire right_publish_ok = (right_track_state != TRACK_STATE_NO_LINE) && right_geom_ok_r;

    assign left_state_o = left_publish_ok ? left_voted_state : 2'd0;
    assign right_state_o = right_publish_ok ? right_voted_state : 2'd0;
    assign left_fit_valid_o = left_fit_valid_raw && left_publish_ok;
    assign right_fit_valid_o = right_fit_valid_raw && right_publish_ok;
    assign debug_left_point_count_o = left_point_count;
    assign debug_right_point_count_o = right_point_count;
    assign debug_left_fit_valid_raw_o = left_fit_valid_raw;
    assign debug_right_fit_valid_raw_o = right_fit_valid_raw;
    assign debug_left_max_error_o = left_max_error;
    assign debug_right_max_error_o = right_max_error;

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state             <= S_IDLE;
            proc_row          <= 5'd0;
            scan_row_sel      <= 5'd0;
            remap_start       <= 1'b0;
            run_row_start     <= 1'b0;
            track_row_start   <= 1'b0;
            track_frame_start <= 1'b0;
            track_frame_end   <= 1'b0;
            fit_frame_start   <= 1'b0;
            fit_frame_end     <= 1'b0;
            gap_frame_done    <= 1'b0;
            geometry_start    <= 1'b0;
            geometry_started  <= 1'b0;
            publish_wait      <= 1'b0;
            left_geom_ok_r    <= 1'b0;
            right_geom_ok_r   <= 1'b0;
            fit_done_l_seen   <= 1'b0;
            fit_done_r_seen   <= 1'b0;
            src_x             <= {X_WIDTH{1'b0}};
            for (i = 0; i < SCAN_ROWS; i = i + 1) begin
                coeff_start[i] <= 32'd0;
                coeff_step[i]  <= 32'd65536;
                left_hits[i]   <= 1'b0;
                right_hits[i]  <= 1'b0;
            end
        end else begin
            remap_start     <= 1'b0;
            run_row_start   <= 1'b0;
            track_row_start <= 1'b0;
            track_frame_start <= 1'b0;
            track_frame_end   <= 1'b0;
            fit_frame_start   <= 1'b0;
            fit_frame_end     <= 1'b0;
            gap_frame_done    <= 1'b0;
            geometry_start    <= 1'b0;

            if (coeff_wr_i && (coeff_row_i < SCAN_ROWS)) begin
                coeff_start[coeff_row_i] <= coeff_start_q16_i;
                coeff_step[coeff_row_i]  <= coeff_step_q16_i;
            end

            if (remap_src_pop)
                src_x <= src_x + 1'b1;

            if (left_fit_done)
                fit_done_l_seen <= 1'b1;
            if (right_fit_done)
                fit_done_r_seen <= 1'b1;

            case (state)
                S_IDLE: begin
                    src_x <= {X_WIDTH{1'b0}};
                    if (frame_done_i) begin
                        proc_row        <= 5'd0;
                        fit_done_l_seen <= 1'b0;
                        fit_done_r_seen <= 1'b0;
                        geometry_started <= 1'b0;
                        publish_wait     <= 1'b0;
                        left_geom_ok_r   <= 1'b0;
                        right_geom_ok_r  <= 1'b0;
                        state           <= S_INIT;
                    end
                end

                S_INIT: begin
                    track_frame_start <= 1'b1;
                    fit_frame_start   <= 1'b1;
                    geometry_started  <= 1'b0;
                    publish_wait      <= 1'b0;
                    left_geom_ok_r    <= 1'b0;
                    right_geom_ok_r   <= 1'b0;
                    state             <= S_ROW_START;
                end

                S_ROW_START: begin
                    remap_start     <= 1'b1;
                    run_row_start   <= 1'b1;
                    track_row_start <= 1'b1;
                    src_x           <= {X_WIDTH{1'b0}};
                    state           <= S_REMAP;
                end

                S_REMAP: begin
                    if (run_row_done)
                        state <= S_ROW_HIT;
                end

                S_ROW_HIT: begin
                    if (left_row_hit_valid || right_row_hit_valid) begin
                        left_hits[proc_row]  <= left_row_hit;
                        right_hits[proc_row] <= right_row_hit;

                        if (proc_row == SCAN_ROWS-1)
                            state <= S_END_PULSE;
                        else begin
                            proc_row <= proc_row + 1'b1;
                            state    <= S_ROW_START;
                        end
                    end
                end

                S_END_PULSE: begin
                    track_frame_end <= 1'b1;
                    fit_frame_end   <= 1'b1;
                    state           <= S_WAIT_FIT;
                end

                S_WAIT_FIT: begin
                    if (!geometry_started) begin
                        if (fit_done_l_seen && fit_done_r_seen) begin
                            geometry_start   <= 1'b1;
                            geometry_started <= 1'b1;
                        end
                    end else if (geom_done) begin
                        left_geom_ok_r  <= left_geom_ok;
                        right_geom_ok_r <= right_geom_ok;
                        gap_frame_done  <= 1'b1;
                        publish_wait    <= 2'd2;
                        state           <= S_PUBLISH;
                    end
                end

                S_PUBLISH: begin
                    if (publish_wait != 2'd0)
                        publish_wait <= publish_wait - 1'b1;
                    else
                        state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    reg result_valid_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            result_valid_r <= 1'b0;
        else
            result_valid_r <= (state == S_PUBLISH) && !publish_wait;
    end

    assign result_valid_o = result_valid_r;
    assign busy_o = (state != S_IDLE);

    // Bring-up activity probes (2026-10-10): per-frame totals, cleared at
    // frame start so a host read shows the activity of the last frame.
    // edge pulses = frontend edge events; run events = run-filter outputs
    // after the MIN/MAX/WIDE gates. Together they pin the dead stage.
    reg [15:0] dbg_edge_cnt;
    reg [15:0] dbg_run_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dbg_edge_cnt <= 16'd0;
            dbg_run_cnt  <= 16'd0;
        end else if (frame_start_i) begin
            dbg_edge_cnt <= 16'd0;
            dbg_run_cnt  <= 16'd0;
        end else begin
            if (edge_valid_i && dbg_edge_cnt != 16'hffff)
                dbg_edge_cnt <= dbg_edge_cnt + 16'd1;
            if (run_valid && dbg_run_cnt != 16'hffff)
                dbg_run_cnt <= dbg_run_cnt + 16'd1;
        end
    end
    assign dbg_edge_pulses_o = dbg_edge_cnt;
    assign dbg_run_events_o  = dbg_run_cnt;

    wire unused_scan_row_sel = (scan_row_sel == 0);
    wire unused_left_track_hits = (left_track_hits == 0);
    wire unused_right_track_hits = (right_track_hits == 0);
    wire unused_left_track_rows = (left_track_rows == 0);
    wire unused_right_track_rows = (right_track_rows == 0);
    wire unused_left_hold = (left_hold_count == 0);
    wire unused_right_hold = (right_hold_count == 0);
    wire unused_left_reacquire = left_reacquire;
    wire unused_right_reacquire = right_reacquire;
    wire unused_left_point_count = (left_point_count == 0);
    wire unused_right_point_count = (right_point_count == 0);
    wire unused_left_gap = left_gap_cov + left_gap_gaps + left_gap_rows;
    wire unused_right_gap = right_gap_cov + right_gap_gaps + right_gap_rows;
    wire unused_remap_busy = remap_busy;
    wire unused_fit_busy = left_fit_busy | right_fit_busy;
    wire unused_result_valid = left_gap_result_valid | right_gap_result_valid;

endmodule
