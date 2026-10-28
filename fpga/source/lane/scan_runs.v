// ============================================================
// scan_runs.v - 扫描行 run 提取
// ============================================================
// 算法来源: ai/scripts/43_车道线检测_逐行重映射.py scan_peaks()
// 参数来源: MIN_RUN=3, MAX_RUN=60 (PC v3)
// 验收判据: tb_scan_runs.v run width/center cases
// 输入是一行重映射后的二值位流。每个有效 bit 对应一个目标列，
// 模块提取宽度在 [MIN_RUN, MAX_RUN] 内的连续 1-run，并输出中心列。
//
// 时序约定：
//   row_start_i 拉高一拍，开始新扫描行。
//   bit_valid_i 连续输入该行 bit_i。
//   row_end_i 在最后一个 bit 之后单独拉高一拍。
//   row_done_o 在所有 run 输出后拉高一拍，供跟踪模块收尾。
// ============================================================
`timescale 1ns/1ps

module scan_runs #(
    parameter integer MIN_RUN = 3,
    parameter integer MAX_RUN = 60
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        row_start_i,
    input  wire [4:0]  row_idx_i,
    input  wire        bit_valid_i,
    input  wire        bit_i,
    input  wire        row_end_i,

    output reg         run_valid_o,
    output reg  [10:0] run_center_o,
    output reg  [7:0]  run_len_o,
    output reg  [4:0]  run_row_idx_o,
    output reg  [7:0]  run_count_o,
    output reg         row_done_o
);

    localparam [1:0] S_IDLE = 2'd0;
    localparam [1:0] S_ROW  = 2'd1;
    localparam [1:0] S_END  = 2'd2;

    reg [1:0]  state;
    reg [10:0] col_idx;
    reg [10:0] run_start;
    reg [7:0]  run_len;
    reg        in_run;
    reg [4:0]  row_idx_lat;

    wire close_on_zero = (state == S_ROW) && bit_valid_i && !bit_i && in_run;
    wire close_on_end  = (state == S_ROW) && row_end_i && in_run;

    wire [10:0] center_from_run = run_start + ((run_len - 1'b1) >> 1);
    wire        range_ok = (run_len >= MIN_RUN) && (run_len <= MAX_RUN);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_IDLE;
            col_idx       <= 11'd0;
            run_start     <= 11'd0;
            run_len       <= 8'd0;
            in_run        <= 1'b0;
            row_idx_lat   <= 5'd0;
            run_valid_o   <= 1'b0;
            run_center_o  <= 11'd0;
            run_len_o     <= 8'd0;
            run_row_idx_o <= 5'd0;
            run_count_o   <= 8'd0;
            row_done_o    <= 1'b0;
        end else begin
            run_valid_o <= 1'b0;
            row_done_o  <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (row_start_i) begin
                        state       <= S_ROW;
                        col_idx     <= 11'd0;
                        run_start   <= 11'd0;
                        run_len     <= 8'd0;
                        in_run      <= 1'b0;
                        row_idx_lat <= row_idx_i;
                        run_count_o <= 8'd0;
                    end
                end

                S_ROW: begin
                    if (bit_valid_i) begin
                        col_idx <= col_idx + 1'b1;

                        if (bit_i) begin
                            if (!in_run) begin
                                in_run    <= 1'b1;
                                run_start <= col_idx;
                                run_len   <= 8'd1;
                            end else begin
                                run_len <= run_len + 1'b1;
                            end
                        end else if (in_run) begin
                            in_run <= 1'b0;
                            if (range_ok) begin
                                run_valid_o   <= 1'b1;
                                run_center_o  <= center_from_run;
                                run_len_o     <= run_len;
                                run_row_idx_o <= row_idx_lat;
                                run_count_o   <= run_count_o + 1'b1;
                            end
                            run_len <= 8'd0;
                        end
                    end

                    if (row_end_i) begin
                        if (in_run) begin
                            if (range_ok) begin
                                run_valid_o   <= 1'b1;
                                run_center_o  <= center_from_run;
                                run_len_o     <= run_len;
                                run_row_idx_o <= row_idx_lat;
                                run_count_o   <= run_count_o + 1'b1;
                            end
                            in_run <= 1'b0;
                        end
                        state <= S_END;
                    end
                end

                S_END: begin
                    row_done_o <= 1'b1;
                    state      <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // Retain the explicit close-on-zero expression for waveform visibility.
    wire unused_close_on_zero = close_on_zero;

endmodule
