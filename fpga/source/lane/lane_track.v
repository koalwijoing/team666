// ============================================================
// lane_track.v - 单侧车道线跟踪三态状态机
// ============================================================
// 算法来源: ai/scripts/43_车道线检测_逐行重映射.py track()
// 参数来源: N_MIN=8, HOLD_MAX=8, window supplied by lane_top
// 验收判据: tb_lane_track.v lock/hold/reacquire + tb_lane_core.v
// 每帧按扫描行顺序输入候选中心点：
//   frame_start_i  一帧开始
//   row_start_i    当前扫描行开始
//   cand_valid_i   当前周期有一个候选中心 cand_x_i
//   row_end_i      当前扫描行候选输入结束
//   frame_end_i    最后一行结束后一拍
//
// 预测：
//   pred_x(row) = fit_b_q8_i + fit_a_q8_i * row_index
//   fit_valid_i=0 时使用 default_x_i，斜率按 0 处理。
//
// 三态：
//   NO_LINE -> LOCKED -> HOLD
//   命中行数达到 N_MIN 后锁定；丢线时最多保持 HOLD_MAX 帧；
//   超过保持期后回 NO_LINE，等待重新捕获。
// ============================================================
`timescale 1ns/1ps

module lane_track #(
    parameter integer N_MIN       = 8,
    parameter integer HOLD_MAX    = 8,
    parameter integer IDEAL_WIDTH = 8
) (
    input  wire               clk,
    input  wire               rst_n,

    input  wire               frame_start_i,
    input  wire               row_start_i,
    input  wire [4:0]         row_idx_i,
    input  wire               cand_valid_i,
    input  wire [10:0]        cand_x_i,
    input  wire [7:0]         cand_width_i,
    input  wire               row_end_i,
    input  wire               frame_end_i,

    input  wire               fit_valid_i,
    input  wire signed [31:0] fit_a_q8_i,
    input  wire signed [31:0] fit_b_q8_i,
    input  wire [10:0]        default_x_i,
    input  wire [10:0]        window_i,

    output reg                row_hit_o,
    output reg                row_hit_valid_o,
    output reg  [10:0]        chosen_x_o,
    output reg  [5:0]         hit_count_o,
    output reg  [5:0]         row_count_o,
    output reg  [1:0]         state_o,
    output reg  [3:0]         hold_count_o,
    output reg                frame_result_valid_o,
    output reg                reacquire_o
);

    localparam [1:0] TRACK_NO_LINE = 2'd0;
    localparam [1:0] TRACK_LOCKED  = 2'd1;
    localparam [1:0] TRACK_HOLD    = 2'd2;

    reg               row_active;
    reg               best_seen;
    reg  [11:0]       best_dist;
    reg  [12:0]       best_cost;
    reg  [10:0]       best_x;
    reg signed [31:0] pred_q8;
    reg signed [31:0] slope_q8;
    reg  [11:0]       pred_x;
    reg  [11:0]       cand_x_ext;
    reg  [11:0]       diff;
    reg  [7:0]        width_penalty;
    reg  [12:0]       cand_cost;

    wire row_end_hit = row_end_i && best_seen;
    wire [5:0] hit_count_after = hit_count_o + (row_end_hit ? 1'b1 : 1'b0);
    wire [5:0] row_count_after = row_count_o + (row_end_i ? 1'b1 : 1'b0);

    // Convert signed Q8.8 prediction to a clamped unsigned pixel coordinate.
    always @(*) begin
        if (pred_q8 <= 0)
            pred_x = 12'd0;
        else if (pred_q8 >= 32'sd524032) // 2047 << 8
            pred_x = 12'd2047;
        else
            pred_x = pred_q8[18:8];
    end

    always @(*) begin
        cand_x_ext = {1'b0, cand_x_i};
        if (cand_x_ext >= pred_x)
            diff = cand_x_ext - pred_x;
        else
            diff = pred_x - cand_x_ext;
    end

    always @(*) begin
        if (cand_width_i >= IDEAL_WIDTH)
            width_penalty = cand_width_i - IDEAL_WIDTH;
        else
            width_penalty = IDEAL_WIDTH - cand_width_i;
        cand_cost = {1'b0, diff} + {5'd0, width_penalty};
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row_active           <= 1'b0;
            best_seen            <= 1'b0;
            best_dist            <= 12'hFFF;
            best_cost            <= 13'h1FFF;
            best_x               <= 11'd0;
            pred_q8              <= 32'sd0;
            slope_q8             <= 32'sd0;
            row_hit_o            <= 1'b0;
            row_hit_valid_o      <= 1'b0;
            chosen_x_o           <= 11'd0;
            hit_count_o          <= 6'd0;
            row_count_o          <= 6'd0;
            state_o              <= TRACK_NO_LINE;
            hold_count_o         <= 4'd0;
            frame_result_valid_o <= 1'b0;
            reacquire_o          <= 1'b0;
        end else begin
            row_hit_valid_o      <= 1'b0;
            frame_result_valid_o <= 1'b0;
            reacquire_o          <= 1'b0;

            if (frame_start_i) begin
                row_active      <= 1'b0;
                best_seen       <= 1'b0;
                best_dist       <= 12'hFFF;
                best_cost       <= 13'h1FFF;
                best_x          <= 11'd0;
                row_hit_o       <= 1'b0;
                hit_count_o     <= 6'd0;
                row_count_o     <= 6'd0;

                if (fit_valid_i) begin
                    pred_q8  <= fit_b_q8_i;
                    slope_q8 <= fit_a_q8_i;
                end else begin
                    pred_q8  <= {default_x_i, 8'b0};
                    slope_q8 <= 32'sd0;
                end
            end

            if (row_start_i) begin
                row_active <= 1'b1;
                best_seen  <= 1'b0;
                best_dist  <= 12'hFFF;
                best_cost  <= 13'h1FFF;
                best_x     <= 11'd0;
            end

            if (cand_valid_i && row_active) begin
                if ((diff <= {1'b0, window_i}) &&
                    (!best_seen || (cand_cost < best_cost))) begin
                    best_seen <= 1'b1;
                    best_dist <= diff;
                    best_cost <= cand_cost;
                    best_x    <= cand_x_i;
                end
            end

            if (row_end_i) begin
                row_active      <= 1'b0;
                row_hit_o       <= best_seen;
                row_hit_valid_o <= 1'b1;
                chosen_x_o      <= best_x;
                row_count_o     <= row_count_after;

                if (best_seen)
                    hit_count_o <= hit_count_after;

                // Rows arrive in increasing order: advance the predicted x.
                pred_q8 <= pred_q8 + slope_q8;
            end

            if (frame_end_i) begin
                if (hit_count_after >= N_MIN) begin
                    if (state_o == TRACK_NO_LINE)
                        reacquire_o <= 1'b1;
                    state_o      <= TRACK_LOCKED;
                    hold_count_o <= 4'd0;
                end else if ((state_o != TRACK_NO_LINE) &&
                             (hold_count_o < HOLD_MAX)) begin
                    state_o      <= TRACK_HOLD;
                    hold_count_o <= hold_count_o + 1'b1;
                end else begin
                    state_o      <= TRACK_NO_LINE;
                    hold_count_o <= 4'd0;
                end

                frame_result_valid_o <= 1'b1;
            end
        end
    end

    // Keep row_idx_i in the interface contract even though prediction advances
    // per row. Integration asserts that rows are delivered in increasing order.
    wire unused_row_idx = (row_idx_i == 5'd0);

endmodule
