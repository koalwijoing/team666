// ============================================================
// lane_result_regs.v - lane result BAR/register window
// ============================================================
// Implements the T-PL-5 result registers at BAR offset 0x1C0.
// The commit input is expected in the destination clock domain.
// ============================================================
`timescale 1ns/1ps

module lane_result_regs #(
    parameter [10:0] DEFAULT_THRESHOLD  = 11'd30,
    parameter [10:0] DEFAULT_LEFT       = 11'd342,
    parameter [10:0] DEFAULT_RIGHT      = 11'd895,
    parameter [10:0] DEFAULT_WINDOW     = 11'd48
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
    input  wire [31:0]          dbg_i,

    output reg                  lane_enable_o,
    output reg                  irq_en_o,
    output reg                  lane_soft_reset_o,
    output reg  [10:0]          threshold_o,
    output reg  [10:0]          expected_left_o,
    output reg  [10:0]          expected_right_o,
    output reg  [10:0]          window_o,
    output wire [31:0]          result_seq_o,

    input  wire [9:0]           wr_addr_i,
    input  wire [9:0]           rd_addr_i,
    input  wire [31:0]          wdata_i,
    input  wire                 wr_i,
    input  wire                 rd_i,
    output reg  [31:0]          rdata_o,
    output reg  [127:0]         rdata_128_o,
    output reg                  ready_o
);

    localparam [9:0] A_CTRL   = 10'h1c0;
    localparam [9:0] A_STATUS = 10'h1c4;
    localparam [9:0] A_SEQ    = 10'h1c8;
    localparam [9:0] A_FRAME  = 10'h1cc;
    localparam [9:0] A_LSTATE = 10'h1d0;
    localparam [9:0] A_LA     = 10'h1d4;
    localparam [9:0] A_LB     = 10'h1d8;
    localparam [9:0] A_RSTATE = 10'h1dc;
    localparam [9:0] A_RA     = 10'h1e0;
    localparam [9:0] A_RB     = 10'h1e4;
    localparam [9:0] A_LPARAM = 10'h1e8;
    localparam [9:0] A_RPARAM = 10'h1ec;

    reg [31:0]          result_seq;
    reg [31:0]          result_frame_id;
    reg [1:0]           result_left_state;
    reg [1:0]           result_right_state;
    reg signed [31:0]   result_left_a_q8;
    reg signed [31:0]   result_right_a_q8;
    reg signed [31:0]   result_left_b_q8;
    reg signed [31:0]   result_right_b_q8;
    reg                 result_left_fit_valid;
    reg                 result_right_fit_valid;
    reg [5:0]           result_left_point_count;
    reg [5:0]           result_right_point_count;
    reg                 result_left_fit_valid_raw;
    reg                 result_right_fit_valid_raw;
    reg [11:0]          result_left_max_error;
    reg [11:0]          result_right_max_error;

    reg                 result_valid_sticky;
    reg                 snapshot_commit_sticky;
    reg                 protocol_error_sticky;

    wire [31:0] ctrl_word = {29'd0, irq_en_o, lane_soft_reset_o, lane_enable_o};
    wire [31:0] status_word = {25'd0,
                               protocol_error_sticky,
                               snapshot_commit_sticky,
                               result_right_fit_valid,
                               result_left_fit_valid,
                               result_valid_sticky,
                               busy_i,
                               lane_enable_o};
    // State words bits[31:22] are unused by every reader; carry the live
    // per-frame debug counters (edge pulses / run events) there so the
    // host can see whether pixels actually reach the lane pipeline.
    wire [31:0] left_state_word =
        {dbg_i[9:0], result_left_max_error, result_left_point_count,
         result_left_fit_valid_raw, result_left_fit_valid, result_left_state};
    wire [31:0] right_state_word =
        {dbg_i[25:16], result_right_max_error, result_right_point_count,
         result_right_fit_valid_raw, result_right_fit_valid, result_right_state};
    wire [31:0] left_param_word  = {5'd0, expected_left_o, 5'd0, threshold_o};
    wire [31:0] right_param_word = {10'd0, window_o, expected_right_o};

    assign result_seq_o = result_seq;

    always @* begin
        rdata_o = 32'd0;
        case (rd_addr_i)
            A_CTRL:   rdata_o = ctrl_word;
            A_STATUS: rdata_o = status_word;
            A_SEQ:    rdata_o = result_seq;
            A_FRAME:  rdata_o = result_frame_id;
            A_LSTATE: rdata_o = left_state_word;
            A_LA:     rdata_o = result_left_a_q8;
            A_LB:     rdata_o = result_left_b_q8;
            A_RSTATE: rdata_o = right_state_word;
            A_RA:     rdata_o = result_right_a_q8;
            A_RB:     rdata_o = result_right_b_q8;
            A_LPARAM: rdata_o = left_param_word;
            A_RPARAM: rdata_o = right_param_word;
            default:  rdata_o = 32'd0;
        endcase
    end

    always @* begin
        rdata_128_o = 128'd0;
        case (rd_addr_i[9:4])
            6'h1c: rdata_128_o = {result_frame_id, result_seq, status_word, ctrl_word};
            6'h1d: rdata_128_o = {right_state_word, result_left_b_q8, result_left_a_q8, left_state_word};
            6'h1e: rdata_128_o = {right_param_word, left_param_word, result_right_b_q8, result_right_a_q8};
            default: rdata_128_o = 128'd0;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lane_enable_o          <= 1'b1;
            irq_en_o               <= 1'b0;
            lane_soft_reset_o      <= 1'b0;
            threshold_o            <= DEFAULT_THRESHOLD;
            expected_left_o        <= DEFAULT_LEFT;
            expected_right_o       <= DEFAULT_RIGHT;
            window_o               <= DEFAULT_WINDOW;
            result_seq             <= 32'd0;
            result_frame_id        <= 32'd0;
            result_left_state      <= 2'd0;
            result_right_state     <= 2'd0;
            result_left_a_q8       <= 32'sd0;
            result_right_a_q8      <= 32'sd0;
            result_left_b_q8       <= 32'sd0;
            result_right_b_q8      <= 32'sd0;
            result_left_fit_valid  <= 1'b0;
            result_right_fit_valid <= 1'b0;
            result_left_point_count <= 6'd0;
            result_right_point_count <= 6'd0;
            result_left_fit_valid_raw <= 1'b0;
            result_right_fit_valid_raw <= 1'b0;
            result_left_max_error <= 12'd0;
            result_right_max_error <= 12'd0;
            result_valid_sticky    <= 1'b0;
            snapshot_commit_sticky <= 1'b0;
            protocol_error_sticky  <= 1'b0;
            ready_o                <= 1'b0;
        end else begin
            lane_soft_reset_o <= 1'b0;
            ready_o           <= rd_i || wr_i;

            if (commit_i) begin
                result_frame_id        <= commit_frame_id_i;
                result_left_state      <= commit_left_state_i;
                result_right_state     <= commit_right_state_i;
                result_left_a_q8       <= commit_left_a_q8_i;
                result_right_a_q8      <= commit_right_a_q8_i;
                result_left_b_q8       <= commit_left_b_q8_i;
                result_right_b_q8      <= commit_right_b_q8_i;
                result_left_fit_valid  <= commit_left_fit_valid_i;
                result_right_fit_valid <= commit_right_fit_valid_i;
                result_left_point_count <= commit_left_point_count_i;
                result_right_point_count <= commit_right_point_count_i;
                result_left_fit_valid_raw <= commit_left_fit_valid_raw_i;
                result_right_fit_valid_raw <= commit_right_fit_valid_raw_i;
                result_left_max_error <= commit_left_max_error_i;
                result_right_max_error <= commit_right_max_error_i;
                result_seq             <= result_seq + 32'd2;
                result_valid_sticky    <= 1'b1;
                snapshot_commit_sticky <= 1'b1;
            end

            if (wr_i) begin
                case (wr_addr_i)
                    A_CTRL: begin
                        lane_enable_o <= wdata_i[0];
                        irq_en_o      <= wdata_i[2];
                        if (wdata_i[1]) begin
                            lane_soft_reset_o      <= 1'b1;
                            result_seq             <= 32'd0;
                            result_left_state      <= 2'd0;
                            result_right_state     <= 2'd0;
                            result_left_a_q8       <= 32'sd0;
                            result_right_a_q8      <= 32'sd0;
                            result_left_b_q8       <= 32'sd0;
                            result_right_b_q8      <= 32'sd0;
                            result_left_fit_valid  <= 1'b0;
                            result_right_fit_valid <= 1'b0;
                            result_left_point_count <= 6'd0;
                            result_right_point_count <= 6'd0;
                            result_left_fit_valid_raw <= 1'b0;
                            result_right_fit_valid_raw <= 1'b0;
                            result_left_max_error <= 12'd0;
                            result_right_max_error <= 12'd0;
                            result_valid_sticky    <= 1'b0;
                            snapshot_commit_sticky <= 1'b0;
                            protocol_error_sticky  <= 1'b0;
                        end
                    end

                    A_STATUS: begin
                        if (wdata_i[2])
                            result_valid_sticky    <= 1'b0;
                        if (wdata_i[3])
                            result_left_fit_valid  <= 1'b0;
                        if (wdata_i[4])
                            result_right_fit_valid <= 1'b0;
                        if (wdata_i[5])
                            snapshot_commit_sticky <= 1'b0;
                        if (wdata_i[6])
                            protocol_error_sticky  <= 1'b0;
                    end

                    A_LPARAM: begin
                        threshold_o     <= wdata_i[10:0];
                        expected_left_o <= wdata_i[26:16];
                    end

                    A_RPARAM: begin
                        expected_right_o <= wdata_i[10:0];
                        window_o         <= wdata_i[21:11];
                    end

                    default: begin
                    end
                endcase
            end

            if (protocol_error_i)
                protocol_error_sticky <= 1'b1;
        end
    end

endmodule
