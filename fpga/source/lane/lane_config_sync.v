// ============================================================
// lane_config_sync.v - static lane configuration synchronization
// ============================================================
// Configuration changes are frame-scoped and quasi-static. Each
// field is synchronized into the pixel domain with two flops.
// ============================================================
`timescale 1ns/1ps

module lane_config_sync (
    input  wire                 src_clk,
    input  wire                 src_rst_n,
    input  wire                 enable_i,
    input  wire [10:0]          threshold_i,
    input  wire [10:0]          expected_left_i,
    input  wire [10:0]          expected_right_i,
    input  wire [10:0]          window_i,

    input  wire                 dst_clk,
    input  wire                 dst_rst_n,
    output reg                  enable_o,
    output reg  [10:0]          threshold_o,
    output reg  [10:0]          expected_left_o,
    output reg  [10:0]          expected_right_o,
    output reg  [10:0]          window_o
);

    (* ASYNC_REG = "TRUE" *) reg enable_meta;
    (* ASYNC_REG = "TRUE" *) reg [10:0] threshold_meta;
    (* ASYNC_REG = "TRUE" *) reg [10:0] expected_left_meta;
    (* ASYNC_REG = "TRUE" *) reg [10:0] expected_right_meta;
    (* ASYNC_REG = "TRUE" *) reg [10:0] window_meta;

    always @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n) begin
            enable_meta         <= 1'b0;
            threshold_meta      <= 11'd30;
            expected_left_meta  <= 11'd342;
            expected_right_meta <= 11'd895;
            window_meta         <= 11'd48;
            enable_o            <= 1'b0;
            threshold_o         <= 11'd30;
            expected_left_o     <= 11'd342;
            expected_right_o    <= 11'd895;
            window_o            <= 11'd48;
        end else begin
            enable_meta         <= enable_i;
            threshold_meta      <= threshold_i;
            expected_left_meta  <= expected_left_i;
            expected_right_meta <= expected_right_i;
            window_meta         <= window_i;
            enable_o            <= enable_meta;
            threshold_o         <= threshold_meta;
            expected_left_o     <= expected_left_meta;
            expected_right_o    <= expected_right_meta;
            window_o            <= window_meta;
        end
    end

endmodule
