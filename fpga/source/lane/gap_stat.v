// ============================================================
// gap_stat.v - 车道线缺口统计与虚实判定
// ============================================================
// 算法来源: ai/scripts/43_车道线检测_逐行重映射.py classify()
// 参数来源: fpga/pl/vectors/README.md, 720p v3 21-row ROI
// 验收判据: tb_gap_stat.v 5 cases + tb_golden_l1.v real-frame L1
// 行流协议：
//   valid_i    : 本拍有效，hit_i 表示该扫描行是否命中
//   hit_i      : 当前扫描行是否命中车道线
//   frame_done : 最后一行的下一拍拉高一拍，触发本帧判定
//
// 默认参数对应 720p v3 原型：
//   ROI 0.60H~0.95H，SCAN_STEP=12 -> 21 条扫描行
//   cov < 0.15       -> NONE
//   gaps >= 2        -> DASHED
//   cov >= 0.62      -> SOLID
//   cov >= 0.40      -> SOLID（遮挡宽容分支）
// ============================================================
`timescale 1ns/1ps

module gap_stat #(
    parameter integer TOTAL_ROWS   = 21,
    parameter integer COV_NONE     = 4,  // 3/21=0.143 < 0.15, 4/21=0.190
    parameter integer COV_OCCLUDED = 9,  // 9/21=0.429 >= 0.40
    parameter integer COV_SOLID    = 14, // 13/21=0.619 < 0.62, 14/21=0.667
    parameter integer GAP_DASH     = 2
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       valid_i,
    input  wire       hit_i,
    input  wire       frame_done,
    output reg  [1:0] state_o,
    output reg  [5:0] cov_o,
    output reg  [5:0] gaps_o,
    output reg  [5:0] row_cnt_o,
    output reg        result_valid
);

    localparam [1:0] S_IDLE  = 2'd0;
    localparam [1:0] S_COUNT = 2'd1;
    localparam [1:0] S_JUDGE = 2'd2;

    localparam [1:0] LINE_NONE   = 2'd0;
    localparam [1:0] LINE_DASHED = 2'd1;
    localparam [1:0] LINE_SOLID  = 2'd2;

    reg [1:0] fsm;
    reg [5:0] hit_cnt;
    reg [5:0] gap_cnt;
    reg       in_gap;

    wire count_row = (fsm == S_COUNT) && valid_i;
    wire open_gap  = count_row && !hit_i && !in_gap;
    wire final_row = count_row || ((fsm == S_IDLE) && valid_i);

    wire [5:0] final_rows = (fsm == S_COUNT) ? (row_cnt_o + (count_row ? 1'b1 : 1'b0)) :
                                               (valid_i ? 6'd1 : 6'd0);
    wire [5:0] final_hits = (fsm == S_COUNT) ? (hit_cnt + ((count_row && hit_i) ? 1'b1 : 1'b0)) :
                                               (valid_i && hit_i ? 6'd1 : 6'd0);
    wire [5:0] final_gaps = (fsm == S_COUNT) ? (gap_cnt + (open_gap ? 1'b1 : 1'b0)) :
                                               (valid_i && !hit_i ? 6'd1 : 6'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fsm          <= S_IDLE;
            hit_cnt      <= 6'd0;
            gap_cnt      <= 6'd0;
            in_gap       <= 1'b0;
            state_o      <= LINE_NONE;
            cov_o        <= 6'd0;
            gaps_o       <= 6'd0;
            row_cnt_o    <= 6'd0;
            result_valid <= 1'b0;
        end else begin
            result_valid <= 1'b0;

            case (fsm)
                S_IDLE: begin
                    if (valid_i) begin
                        hit_cnt   <= hit_i ? 6'd1 : 6'd0;
                        gap_cnt   <= hit_i ? 6'd0 : 6'd1;
                        in_gap    <= !hit_i;
                        row_cnt_o <= 6'd1;
                        fsm       <= S_COUNT;
                    end
                end

                S_COUNT: begin
                    if (valid_i) begin
                        row_cnt_o <= row_cnt_o + 1'b1;
                        if (hit_i) begin
                            hit_cnt <= hit_cnt + 1'b1;
                            in_gap  <= 1'b0;
                        end else if (!in_gap) begin
                            gap_cnt <= gap_cnt + 1'b1;
                            in_gap  <= 1'b1;
                        end
                    end

                    if (frame_done) begin
                        cov_o        <= final_hits;
                        gaps_o       <= final_gaps;
                        row_cnt_o    <= final_rows;
                        fsm          <= S_JUDGE;
                    end
                end

                S_JUDGE: begin
                    if (cov_o < COV_NONE)
                        state_o <= LINE_NONE;
                    else if (gaps_o >= GAP_DASH)
                        state_o <= LINE_DASHED;
                    else if (cov_o >= COV_SOLID)
                        state_o <= LINE_SOLID;
                    else if (cov_o >= COV_OCCLUDED)
                        state_o <= LINE_SOLID;
                    else
                        state_o <= LINE_NONE;

                    result_valid <= 1'b1;
                    fsm          <= S_IDLE;
                end

                default: fsm <= S_IDLE;
            endcase
        end
    end

    // Keep the parameter visible to lint and documentation tools.
    wire unused_total_rows = (TOTAL_ROWS == 0);

endmodule
