// ============================================================
// lane_top.v - complete FPGA lane detection top
// ============================================================
// 算法来源: ai/scripts/43_车道线检测_逐行重映射.py full deployment path
// 参数来源: fpga/pl/vectors/README.md and lane_core defaults
// 验收判据: tb_lane_top.v full RGB565 top integration
// Input : RGB565 video stream
// Output: voted left/right line state and Q8.8 line coefficients
// ============================================================
`timescale 1ns/1ps

module lane_top #(
    parameter integer WIDTH       = 1280,
    parameter integer X_WIDTH     = 11,
    parameter integer Y_WIDTH     = 11,
    parameter integer SCAN_ROWS   = 21,
    parameter integer SCAN_Y0     = 432,
    parameter integer SCAN_STEP   = 12,
    parameter integer MIN_RUN     = 3,
    parameter integer MAX_RUN     = 60,
    parameter integer N_MIN       = 8,
    parameter integer HOLD_MAX    = 8,
    parameter integer MIN_POINTS  = 2,
    parameter integer COV_NONE     = 4,
    parameter integer COV_OCCLUDED = 9,
    parameter integer COV_SOLID    = 14,
    parameter integer GAP_DASH     = 2
) (
    input  wire                 clk,
    input  wire                 rst_n,

    input  wire                 frame_start_i,
    input  wire                 valid_i,
    input  wire                 eol_i,
    input  wire [15:0]          rgb565_i,
    input  wire                 frame_done_i,
    input  wire [10:0]          threshold_i,

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

    wire edge_valid;
    wire edge_bit;
    wire [X_WIDTH-1:0] edge_x;
    wire [Y_WIDTH-1:0] edge_y;
    wire edge_eol;

    lane_frontend #(
        .WIDTH(WIDTH),
        .X_WIDTH(X_WIDTH),
        .Y_WIDTH(Y_WIDTH)
    ) u_frontend (
        .clk(clk),
        .rst_n(rst_n),
        .frame_start_i(frame_start_i),
        .valid_i(valid_i),
        .eol_i(eol_i),
        .rgb565_i(rgb565_i),
        .threshold_i(threshold_i),
        .edge_valid_o(edge_valid),
        .edge_bit_o(edge_bit),
        .x_o(edge_x),
        .y_o(edge_y),
        .eol_o(edge_eol)
    );

    lane_core #(
        .WIDTH(WIDTH),
        .X_WIDTH(X_WIDTH),
        .Y_WIDTH(Y_WIDTH),
        .SCAN_ROWS(SCAN_ROWS),
        .SCAN_Y0(SCAN_Y0),
        .SCAN_STEP(SCAN_STEP),
        .MIN_RUN(MIN_RUN),
        .MAX_RUN(MAX_RUN),
        .N_MIN(N_MIN),
        .HOLD_MAX(HOLD_MAX),
        .MIN_POINTS(MIN_POINTS),
        .COV_NONE(COV_NONE),
        .COV_OCCLUDED(COV_OCCLUDED),
        .COV_SOLID(COV_SOLID),
        .GAP_DASH(GAP_DASH)
    ) u_core (
        .clk(clk),
        .rst_n(rst_n),
        .frame_start_i(frame_start_i),
        .edge_valid_i(edge_valid),
        .edge_bit_i(edge_bit),
        .x_i(edge_x),
        .y_i(edge_y),
        .eol_i(edge_eol),
        .frame_done_i(frame_done_i),
        .coeff_wr_i(coeff_wr_i),
        .coeff_row_i(coeff_row_i),
        .coeff_start_q16_i(coeff_start_q16_i),
        .coeff_step_q16_i(coeff_step_q16_i),
        .expected_left_i(expected_left_i),
        .expected_right_i(expected_right_i),
        .window_i(window_i),
        .left_state_o(left_state_o),
        .right_state_o(right_state_o),
        .left_a_q8_o(left_a_q8_o),
        .right_a_q8_o(right_a_q8_o),
        .left_b_q8_o(left_b_q8_o),
        .right_b_q8_o(right_b_q8_o),
        .left_fit_valid_o(left_fit_valid_o),
        .right_fit_valid_o(right_fit_valid_o),
        .debug_left_point_count_o(debug_left_point_count_o),
        .debug_right_point_count_o(debug_right_point_count_o),
        .debug_left_fit_valid_raw_o(debug_left_fit_valid_raw_o),
        .debug_right_fit_valid_raw_o(debug_right_fit_valid_raw_o),
        .debug_left_max_error_o(debug_left_max_error_o),
        .debug_right_max_error_o(debug_right_max_error_o),
        .dbg_edge_pulses_o(dbg_edge_pulses_o),
        .dbg_run_events_o(dbg_run_events_o),
        .result_valid_o(result_valid_o),
        .busy_o(busy_o)
    );

endmodule
