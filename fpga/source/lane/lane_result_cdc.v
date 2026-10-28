// ============================================================
// lane_result_cdc.v - lane result snapshot across clock domains
// ============================================================
// The source domain captures one coherent result snapshot and
// toggles a handshake bit. The destination domain samples the
// stable snapshot after synchronizing the toggle.
// ============================================================
`timescale 1ns/1ps

module lane_result_cdc (
    input  wire                 src_clk,
    input  wire                 src_rst_n,
    input  wire                 src_commit_i,
    input  wire [31:0]          src_frame_id_i,
    input  wire [1:0]           src_left_state_i,
    input  wire [1:0]           src_right_state_i,
    input  wire signed [31:0]   src_left_a_q8_i,
    input  wire signed [31:0]   src_right_a_q8_i,
    input  wire signed [31:0]   src_left_b_q8_i,
    input  wire signed [31:0]   src_right_b_q8_i,
    input  wire                 src_left_fit_valid_i,
    input  wire                 src_right_fit_valid_i,
    input  wire [5:0]           src_left_point_count_i,
    input  wire [5:0]           src_right_point_count_i,
    input  wire                 src_left_fit_valid_raw_i,
    input  wire                 src_right_fit_valid_raw_i,
    input  wire [11:0]          src_left_max_error_i,
    input  wire [11:0]          src_right_max_error_i,

    input  wire                 dst_clk,
    input  wire                 dst_rst_n,

    output reg                  dst_commit_o,
    output reg  [31:0]          dst_frame_id_o,
    output reg  [1:0]           dst_left_state_o,
    output reg  [1:0]           dst_right_state_o,
    output reg  signed [31:0]   dst_left_a_q8_o,
    output reg  signed [31:0]   dst_right_a_q8_o,
    output reg  signed [31:0]   dst_left_b_q8_o,
    output reg  signed [31:0]   dst_right_b_q8_o,
    output reg                  dst_left_fit_valid_o,
    output reg                  dst_right_fit_valid_o,
    output reg  [5:0]           dst_left_point_count_o,
    output reg  [5:0]           dst_right_point_count_o,
    output reg                  dst_left_fit_valid_raw_o,
    output reg                  dst_right_fit_valid_raw_o,
    output reg  [11:0]          dst_left_max_error_o,
    output reg  [11:0]          dst_right_max_error_o
);

    reg                 toggle_src;
    reg [31:0]          snap_frame_id;
    reg [1:0]           snap_left_state;
    reg [1:0]           snap_right_state;
    reg signed [31:0]   snap_left_a_q8;
    reg signed [31:0]   snap_right_a_q8;
    reg signed [31:0]   snap_left_b_q8;
    reg signed [31:0]   snap_right_b_q8;
    reg                 snap_left_fit_valid;
    reg                 snap_right_fit_valid;
    reg [5:0]           snap_left_point_count;
    reg [5:0]           snap_right_point_count;
    reg                 snap_left_fit_valid_raw;
    reg                 snap_right_fit_valid_raw;
    reg [11:0]          snap_left_max_error;
    reg [11:0]          snap_right_max_error;

    (* ASYNC_REG = "TRUE" *) reg toggle_meta;
    (* ASYNC_REG = "TRUE" *) reg toggle_sync;
    reg toggle_seen;

    always @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n) begin
            toggle_src           <= 1'b0;
            snap_frame_id        <= 32'd0;
            snap_left_state      <= 2'd0;
            snap_right_state     <= 2'd0;
            snap_left_a_q8       <= 32'sd0;
            snap_right_a_q8      <= 32'sd0;
            snap_left_b_q8       <= 32'sd0;
            snap_right_b_q8      <= 32'sd0;
            snap_left_fit_valid  <= 1'b0;
            snap_right_fit_valid <= 1'b0;
            snap_left_point_count <= 6'd0;
            snap_right_point_count <= 6'd0;
            snap_left_fit_valid_raw <= 1'b0;
            snap_right_fit_valid_raw <= 1'b0;
            snap_left_max_error <= 12'd0;
            snap_right_max_error <= 12'd0;
        end else if (src_commit_i) begin
            toggle_src           <= !toggle_src;
            snap_frame_id        <= src_frame_id_i;
            snap_left_state      <= src_left_state_i;
            snap_right_state     <= src_right_state_i;
            snap_left_a_q8       <= src_left_a_q8_i;
            snap_right_a_q8      <= src_right_a_q8_i;
            snap_left_b_q8       <= src_left_b_q8_i;
            snap_right_b_q8      <= src_right_b_q8_i;
            snap_left_fit_valid  <= src_left_fit_valid_i;
            snap_right_fit_valid <= src_right_fit_valid_i;
            snap_left_point_count <= src_left_point_count_i;
            snap_right_point_count <= src_right_point_count_i;
            snap_left_fit_valid_raw <= src_left_fit_valid_raw_i;
            snap_right_fit_valid_raw <= src_right_fit_valid_raw_i;
            snap_left_max_error <= src_left_max_error_i;
            snap_right_max_error <= src_right_max_error_i;
        end
    end

    always @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n) begin
            toggle_meta          <= 1'b0;
            toggle_sync          <= 1'b0;
            toggle_seen          <= 1'b0;
            dst_commit_o         <= 1'b0;
            dst_frame_id_o       <= 32'd0;
            dst_left_state_o     <= 2'd0;
            dst_right_state_o    <= 2'd0;
            dst_left_a_q8_o      <= 32'sd0;
            dst_right_a_q8_o     <= 32'sd0;
            dst_left_b_q8_o      <= 32'sd0;
            dst_right_b_q8_o     <= 32'sd0;
            dst_left_fit_valid_o <= 1'b0;
            dst_right_fit_valid_o <= 1'b0;
            dst_left_point_count_o <= 6'd0;
            dst_right_point_count_o <= 6'd0;
            dst_left_fit_valid_raw_o <= 1'b0;
            dst_right_fit_valid_raw_o <= 1'b0;
            dst_left_max_error_o <= 12'd0;
            dst_right_max_error_o <= 12'd0;
        end else begin
            toggle_meta <= toggle_src;
            toggle_sync <= toggle_meta;
            dst_commit_o <= 1'b0;

            if (toggle_sync != toggle_seen) begin
                toggle_seen          <= toggle_sync;
                dst_commit_o         <= 1'b1;
                dst_frame_id_o       <= snap_frame_id;
                dst_left_state_o     <= snap_left_state;
                dst_right_state_o    <= snap_right_state;
                dst_left_a_q8_o      <= snap_left_a_q8;
                dst_right_a_q8_o     <= snap_right_a_q8;
                dst_left_b_q8_o      <= snap_left_b_q8;
                dst_right_b_q8_o     <= snap_right_b_q8;
                dst_left_fit_valid_o <= snap_left_fit_valid;
                dst_right_fit_valid_o <= snap_right_fit_valid;
                dst_left_point_count_o <= snap_left_point_count;
                dst_right_point_count_o <= snap_right_point_count;
                dst_left_fit_valid_raw_o <= snap_left_fit_valid_raw;
                dst_right_fit_valid_raw_o <= snap_right_fit_valid_raw;
                dst_left_max_error_o <= snap_left_max_error;
                dst_right_max_error_o <= snap_right_max_error;
            end
        end
    end

endmodule
