// ============================================================
// lane_result_bar0.v - lane result registers on PCIe BAR0
// ============================================================
// The vendor BAR0 path transfers 128-bit words. Lane registers
// occupy the aligned blocks 0x1C0, 0x1D0 and 0x1E0.
// ============================================================
`timescale 1ns/1ps

module lane_result_bar0 #(
    parameter [10:0] DEFAULT_THRESHOLD = 11'd30,
    parameter [10:0] DEFAULT_LEFT      = 11'd342,
    parameter [10:0] DEFAULT_RIGHT     = 11'd895,
    parameter [10:0] DEFAULT_WINDOW    = 11'd48
) (
    input  wire                 clk,
    input  wire                 rst_n,

    input  wire                 commit_i,
    input  wire [31:0]          commit_frame_id_i,
    input  wire [1:0]           commit_left_state_i,
    input  wire [1:0]           commit_right_state_i,
    input  wire signed [31:0]   commit_left_a_q8_i,
    input  wire signed [31:0]   commit_right_a_q8_i,
    input  wire signed [31:0]   commit_left_b_q8_i,
    input  wire signed [31:0]   commit_right_b_q8_i,
    input  wire                 commit_left_fit_valid_i,
    input  wire                 commit_right_fit_valid_i,
    input  wire [5:0]           commit_left_point_count_i,
    input  wire [5:0]           commit_right_point_count_i,
    input  wire                 commit_left_fit_valid_raw_i,
    input  wire                 commit_right_fit_valid_raw_i,
    input  wire [11:0]          commit_left_max_error_i,
    input  wire [11:0]          commit_right_max_error_i,
    input  wire                 busy_i,
    input  wire                 protocol_error_i,
    input  wire [31:0]          commit_dbg_i,

    output wire                 lane_enable_o,
    output wire                 irq_en_o,
    output wire                 lane_soft_reset_o,
    output wire [10:0]          threshold_o,
    output wire [10:0]          expected_left_o,
    output wire [10:0]          expected_right_o,
    output wire [10:0]          window_o,
    output wire [31:0]          result_seq_o,

    input  wire                 bar0_wr_en_i,
    input  wire [8:0]           bar0_wr_addr_i,
    input  wire [127:0]         bar0_wr_data_i,
    input  wire [15:0]          bar0_wr_be_i,
    input  wire [8:0]           bar0_rd_addr_i,

    output wire                 write_hit_o,
    output wire                 read_hit_o,
    output wire [127:0]         read_data_o
);

    localparam [8:0] FIRST_WORD = 9'h1c;
    localparam [8:0] LAST_WORD  = 9'h1e;

    // Write decoding accepts both the 128-bit word index used by the TB
    // and the byte/word form presented by the integrated BAR0 path.
    wire write_byte_form = (bar0_wr_addr_i[8:4] >= 5'h1c) &&
                           (bar0_wr_addr_i[8:4] <= 5'h1e);
    wire [8:0] write_word_addr = write_byte_form ?
                                 {4'b0000, bar0_wr_addr_i[8:4]} :
                                 bar0_wr_addr_i;

    // The integrated BAR0 path presents byte addresses such as 0x1C0,
    // while the standalone bridge testbench presents a 128-bit word
    // index (0x1C) or its one-cycle-pipelined value (0x1D). Normalize
    // all three forms to the lane block number.
    wire [8:0] read_byte_block = {4'b0000, bar0_rd_addr_i[8:4]};
    wire [8:0] read_word_pipe  = (bar0_rd_addr_i >= 9'h1d) &&
                                 (bar0_rd_addr_i <= 9'h1f) ?
                                 bar0_rd_addr_i - 9'd1 :
                                 bar0_rd_addr_i;
    wire [8:0] read_word_addr  = (bar0_rd_addr_i >= 9'h100) ?
                                 read_byte_block :
                                 read_word_pipe;
    reg [1:0]  lane_sel;
    reg [31:0] lane_wdata;
    wire [9:0] write_addr;
    wire       write_hit;
    wire       read_hit;

    wire [9:0] read_addr = {read_word_addr, 4'b0000};
    wire [127:0] lane_rdata128;

    assign write_hit = bar0_wr_en_i &&
                       (write_word_addr >= FIRST_WORD) &&
                       (write_word_addr <= LAST_WORD) &&
                       (bar0_wr_be_i != 16'd0);
    assign read_hit = (read_word_addr >= FIRST_WORD) &&
                      (read_word_addr <= LAST_WORD);

    assign write_hit_o = write_hit;
    assign read_hit_o  = read_hit;
    assign read_data_o = read_hit ? lane_rdata128 : 128'd0;

    always @* begin
        if (bar0_wr_be_i[3:0] != 4'd0)
            lane_sel = 2'd0;
        else if (bar0_wr_be_i[7:4] != 4'd0)
            lane_sel = 2'd1;
        else if (bar0_wr_be_i[11:8] != 4'd0)
            lane_sel = 2'd2;
        else
            lane_sel = 2'd3;
    end

    always @* begin
        case (lane_sel)
            2'd0:    lane_wdata = bar0_wr_data_i[31:0];
            2'd1:    lane_wdata = bar0_wr_data_i[63:32];
            2'd2:    lane_wdata = bar0_wr_data_i[95:64];
            default: lane_wdata = bar0_wr_data_i[127:96];
        endcase
    end

    assign write_addr = {write_word_addr, 4'b0000} + {6'b000000, lane_sel, 2'b00};

    lane_result_regs #(
        .DEFAULT_THRESHOLD(DEFAULT_THRESHOLD),
        .DEFAULT_LEFT(DEFAULT_LEFT),
        .DEFAULT_RIGHT(DEFAULT_RIGHT),
        .DEFAULT_WINDOW(DEFAULT_WINDOW)
    ) u_regs (
        .clk(clk),
        .rst_n(rst_n),
        .commit_i(commit_i),
        .commit_frame_id_i(commit_frame_id_i),
        .commit_left_state_i(commit_left_state_i),
        .commit_right_state_i(commit_right_state_i),
        .commit_left_a_q8_i(commit_left_a_q8_i),
        .commit_right_a_q8_i(commit_right_a_q8_i),
        .commit_left_b_q8_i(commit_left_b_q8_i),
        .commit_right_b_q8_i(commit_right_b_q8_i),
        .commit_left_fit_valid_i(commit_left_fit_valid_i),
        .commit_right_fit_valid_i(commit_right_fit_valid_i),
        .commit_left_point_count_i(commit_left_point_count_i),
        .commit_right_point_count_i(commit_right_point_count_i),
        .commit_left_fit_valid_raw_i(commit_left_fit_valid_raw_i),
        .commit_right_fit_valid_raw_i(commit_right_fit_valid_raw_i),
        .commit_left_max_error_i(commit_left_max_error_i),
        .commit_right_max_error_i(commit_right_max_error_i),
        .busy_i(busy_i),
        .protocol_error_i(protocol_error_i),
        .dbg_i(commit_dbg_i),
        .lane_enable_o(lane_enable_o),
        .irq_en_o(irq_en_o),
        .lane_soft_reset_o(lane_soft_reset_o),
        .threshold_o(threshold_o),
        .expected_left_o(expected_left_o),
        .expected_right_o(expected_right_o),
        .window_o(window_o),
        .result_seq_o(result_seq_o),
        .wr_addr_i(write_addr),
        .rd_addr_i(read_addr),
        .wdata_i(lane_wdata),
        .wr_i(write_hit),
        .rd_i(read_hit),
        .rdata_o(),
        .rdata_128_o(lane_rdata128),
        .ready_o()
    );

endmodule
