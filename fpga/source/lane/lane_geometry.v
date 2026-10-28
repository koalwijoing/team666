// ============================================================
// lane_geometry.v - per-frame lane geometry and mirror validation
// ============================================================
// The line fit output is in the remapped scan-row coordinate system:
//   x_q8 = a_q8 * row_index + b_q8
// This module rejects fits that cannot correspond to the ego-lane
// boundary. It is intentionally small and purely comparison based.
// ============================================================
`timescale 1ns/1ps

module lane_geometry #(
    parameter integer WIDTH         = 1280,
    parameter integer SCAN_ROWS     = 21,
    // Recalibrated 2026-10-10 against a full-coverage board lock
    // (21/19 scan rows, fit raw valid, max_err 34-37, |slope| up to 558,
    // far-end b ~26% of width). Values carry ~1.4-1.5x margin over that
    // measured lock so valid geometry passes while the temporal votes and
    // mirror check keep rejecting erratic fits.
    // 2026-10-11: opened wide for the current "camera films the road video"
    // framing (the empirical per-row calibration still leaves the tracked
    // dividers slanted 20-90 dst-px/step and off-center by tens of px;
    // residual still rejects noise fits). Re-tighten after a clean
    // optical re-calibration.
    parameter integer MAX_RESIDUAL  = 160,
    parameter integer MAX_SLOPE_Q8  = 32768,
    parameter integer FAR_LO_PCT    = 5,
    parameter integer FAR_HI_PCT    = 95,
    parameter integer MIRROR_TOL_PCT = 9
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start_i,

    input  wire               left_fit_valid_i,
    input  wire signed [31:0] left_a_q8_i,
    input  wire signed [31:0] left_b_q8_i,
    input  wire [11:0]        left_max_error_i,

    input  wire               right_fit_valid_i,
    input  wire signed [31:0] right_a_q8_i,
    input  wire signed [31:0] right_b_q8_i,
    input  wire [11:0]        right_max_error_i,

    output reg                left_ok_o,
    output reg                right_ok_o,
    output reg                done_o
);

    localparam signed [63:0] FAR_LO_Q8 =
        (WIDTH * FAR_LO_PCT * 256) / 100;
    localparam signed [63:0] FAR_HI_Q8 =
        (WIDTH * FAR_HI_PCT * 256) / 100;
    localparam signed [63:0] CENTER_Q8 =
        ((WIDTH - 1) * 256) / 2;
    localparam signed [63:0] MIRROR_TOL_Q8 =
        (WIDTH * MIRROR_TOL_PCT * 256) / 100;
    localparam signed [63:0] NEAR_MAX_Q8 =
        (WIDTH - 1) * 256;
    localparam integer NEAR_ROW = SCAN_ROWS - 1;

    function signed [63:0] scale_by_near_row;
        input signed [31:0] value;
        integer k;
        begin
            scale_by_near_row = 64'sd0;
            for (k = 0; k < 32; k = k + 1) begin
                if ((NEAR_ROW & (1 << k)) != 0)
                    scale_by_near_row = scale_by_near_row +
                                        ($signed(value) <<< k);
            end
        end
    endfunction

    wire signed [63:0] left_near_q8 =
        scale_by_near_row(left_a_q8_i) + left_b_q8_i;
    wire signed [63:0] right_near_q8 =
        scale_by_near_row(right_a_q8_i) + right_b_q8_i;
    wire signed [63:0] near_sum_q8 = left_near_q8 + right_near_q8;
    wire signed [63:0] mirror_error_q8 =
        near_sum_q8 >= (CENTER_Q8 * 2) ? near_sum_q8 - (CENTER_Q8 * 2) :
                                        (CENTER_Q8 * 2) - near_sum_q8;
    wire signed [63:0] left_abs_a =
        left_a_q8_i[31] ? -left_a_q8_i : left_a_q8_i;
    wire signed [63:0] right_abs_a =
        right_a_q8_i[31] ? -right_a_q8_i : right_a_q8_i;

    wire left_far_ok = (left_b_q8_i >= FAR_LO_Q8) &&
                       (left_b_q8_i <= FAR_HI_Q8);
    wire right_far_ok = (right_b_q8_i >= FAR_LO_Q8) &&
                        (right_b_q8_i <= FAR_HI_Q8);
    wire left_near_ok = (left_near_q8 >= 0) &&
                        (left_near_q8 <= NEAR_MAX_Q8) &&
                        (left_near_q8 < CENTER_Q8);
    wire right_near_ok = (right_near_q8 >= 0) &&
                         (right_near_q8 <= NEAR_MAX_Q8) &&
                         (right_near_q8 > CENTER_Q8);
    wire left_slope_ok = left_abs_a <= MAX_SLOPE_Q8;
    wire right_slope_ok = right_abs_a <= MAX_SLOPE_Q8;
    wire left_residual_ok = left_max_error_i <= MAX_RESIDUAL;
    wire right_residual_ok = right_max_error_i <= MAX_RESIDUAL;
    wire mirror_ok = mirror_error_q8 <= MIRROR_TOL_Q8;

    wire left_side_ok = left_fit_valid_i && left_far_ok &&
                        left_near_ok && left_slope_ok && left_residual_ok;
    wire right_side_ok = right_fit_valid_i && right_far_ok &&
                         right_near_ok && right_slope_ok && right_residual_ok;
    wire both_valid = left_side_ok && right_side_ok;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            left_ok_o  <= 1'b0;
            right_ok_o <= 1'b0;
            done_o     <= 1'b0;
        end else begin
            done_o <= 1'b0;
            if (start_i) begin
                if (both_valid && !mirror_ok) begin
                    left_ok_o  <= 1'b0;
                    right_ok_o <= 1'b0;
                end else begin
                    left_ok_o  <= left_side_ok;
                    right_ok_o <= right_side_ok;
                end
                done_o <= 1'b1;
            end
        end
    end

endmodule
