// ============================================================
// line_fit.v - 车道线逐帧最小二乘拟合
// ============================================================
// 算法来源: ai/scripts/43_车道线检测_逐行重映射.py polyfit
// 参数来源: MIN_POINTS and Q8.8 interface in lane_core
// 验收判据: tb_line_fit.v positive/negative/constant lines
// 输入命中点 (row_idx, x)，在 frame_end_i 后计算：
//   x = a * row + b
// 输出 a_q8_o / b_q8_o 为 Q8.8 定点数。
//
// 计算在帧尾串行进行，不阻塞扫描行输入阶段。
// ============================================================
`timescale 1ns/1ps

module line_fit #(
    parameter integer MIN_POINTS = 2,
    parameter integer MAX_POINTS = 21
) (
    input  wire               clk,
    input  wire               rst_n,

    input  wire               frame_start_i,
    input  wire               point_valid_i,
    input  wire [4:0]         row_idx_i,
    input  wire [10:0]        x_i,
    input  wire               frame_end_i,

    output reg  [5:0]         point_count_o,
    output reg signed [31:0]  a_q8_o,
    output reg signed [31:0]  b_q8_o,
    output reg  [11:0]        max_error_o,
    output reg                fit_valid_o,
    output reg                busy_o,
    output reg                done_o
);

    localparam [2:0] S_IDLE   = 3'd0;
    localparam [2:0] S_DIV_A  = 3'd1;
    localparam [2:0] S_CALC_B = 3'd2;
    localparam [2:0] S_DIV_B  = 3'd3;
    localparam [2:0] S_DONE   = 3'd4;
    localparam [2:0] S_RESID  = 3'd5;

    reg [2:0]         state;
    reg [5:0]         point_count;
    reg [5:0]         resid_idx;
    reg signed [47:0] sum_x;
    reg signed [47:0] sum_y;
    reg signed [47:0] sum_xx;
    reg signed [47:0] sum_xy;
    reg [4:0]         point_row [0:MAX_POINTS-1];
    reg [10:0]        point_x   [0:MAX_POINTS-1];
    reg [11:0]        max_error;

    reg signed [63:0] den;
    reg signed [63:0] num;
    reg               sign_a;
    reg               sign_b;
    reg [55:0]        div_a_dividend;
    reg [55:0]        div_b_dividend;
    reg [31:0]        div_a_divisor;
    reg [31:0]        div_b_divisor;

    reg               div_a_start;
    reg               div_b_start;
    wire [55:0]       div_a_quotient;
    wire [31:0]       div_a_remainder;
    wire              div_a_busy;
    wire              div_a_done;

    wire [55:0]       div_b_quotient;
    wire [31:0]       div_b_remainder;
    wire              div_b_busy;
    wire              div_b_done;

    reg signed [63:0] b_num;
    integer i;

    wire [4:0]  resid_read_idx =
        (resid_idx < MAX_POINTS) ? resid_idx[4:0] : 5'd0;
    wire [4:0]  resid_row = point_row[resid_read_idx];
    wire [10:0] resid_x   = point_x[resid_read_idx];
    wire signed [63:0] resid_a_x1 = {{32{a_q8_o[31]}}, a_q8_o};
    wire signed [63:0] resid_a_x2 = resid_a_x1 <<< 1;
    wire signed [63:0] resid_a_x4 = resid_a_x1 <<< 2;
    wire signed [63:0] resid_a_x8 = resid_a_x1 <<< 3;
    wire signed [63:0] resid_a_x16 = resid_a_x1 <<< 4;
    reg  signed [63:0] resid_product;

    always @(*) begin
        resid_product = 64'sd0;
        if (resid_row[0]) resid_product = resid_product + resid_a_x1;
        if (resid_row[1]) resid_product = resid_product + resid_a_x2;
        if (resid_row[2]) resid_product = resid_product + resid_a_x4;
        if (resid_row[3]) resid_product = resid_product + resid_a_x8;
        if (resid_row[4]) resid_product = resid_product + resid_a_x16;
    end

    wire signed [63:0] resid_pred_q8 = resid_product + b_q8_o;
    wire [10:0] resid_pred_x =
        (resid_pred_q8 <= 0) ? 11'd0 :
        (resid_pred_q8 >= (64'sd2047 << 8)) ? 11'd2047 :
        resid_pred_q8[18:8];
    wire [11:0] resid_error =
        (resid_x >= resid_pred_x) ? (resid_x - resid_pred_x) :
                                    (resid_pred_x - resid_x);
    wire [11:0] max_error_next =
        (resid_error > max_error) ? resid_error : max_error;

    seq_div_unsigned u_div_a (
        .clk        (clk),
        .rst_n      (rst_n),
        .start_i    (div_a_start),
        .dividend_i (div_a_dividend),
        .divisor_i  (div_a_divisor),
        .quotient_o (div_a_quotient),
        .remainder_o(div_a_remainder),
        .busy_o     (div_a_busy),
        .done_o     (div_a_done)
    );

    seq_div_unsigned u_div_b (
        .clk        (clk),
        .rst_n      (rst_n),
        .start_i    (div_b_start),
        .dividend_i (div_b_dividend),
        .divisor_i  (div_b_divisor),
        .quotient_o (div_b_quotient),
        .remainder_o(div_b_remainder),
        .busy_o     (div_b_busy),
        .done_o     (div_b_done)
    );

    wire signed [63:0] count_ext = $signed({58'd0, point_count});
    wire signed [63:0] den_eval   = count_ext * sum_xx - sum_x * sum_x;
    wire signed [63:0] num_eval   = count_ext * sum_xy - sum_x * sum_y;
    wire signed [63:0] num_eval_abs = num_eval[63] ? -num_eval : num_eval;
    wire signed [63:0] a_q8_next =
        sign_a ? -$signed(div_a_quotient[31:0]) :
                 $signed(div_a_quotient[31:0]);
    wire signed [63:0] b_num_next =
        (sum_y << 8) - (a_q8_next * sum_x);
    wire signed [63:0] b_num_next_abs =
        b_num_next[63] ? -b_num_next : b_num_next;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state          <= S_IDLE;
            point_count    <= 6'd0;
            resid_idx      <= 6'd0;
            sum_x          <= 48'sd0;
            sum_y          <= 48'sd0;
            sum_xx         <= 48'sd0;
            sum_xy         <= 48'sd0;
            max_error      <= 12'd0;
            for (i = 0; i < MAX_POINTS; i = i + 1) begin
                point_row[i] <= 5'd0;
                point_x[i]   <= 11'd0;
            end
            den            <= 64'sd0;
            num            <= 64'sd0;
            sign_a         <= 1'b0;
            sign_b         <= 1'b0;
            div_a_dividend <= 56'd0;
            div_b_dividend <= 56'd0;
            div_a_divisor  <= 32'd0;
            div_b_divisor  <= 32'd0;
            div_a_start    <= 1'b0;
            div_b_start    <= 1'b0;
            b_num          <= 64'sd0;
            point_count_o  <= 6'd0;
            a_q8_o         <= 32'sd0;
            b_q8_o         <= 32'sd0;
            max_error_o    <= 12'd0;
            fit_valid_o    <= 1'b0;
            busy_o         <= 1'b0;
            done_o         <= 1'b0;
        end else begin
            div_a_start <= 1'b0;
            div_b_start <= 1'b0;
            done_o      <= 1'b0;

            if (frame_start_i) begin
                state         <= S_IDLE;
                point_count   <= 6'd0;
                resid_idx     <= 6'd0;
                sum_x         <= 48'sd0;
                sum_y         <= 48'sd0;
                sum_xx        <= 48'sd0;
                sum_xy        <= 48'sd0;
                max_error     <= 12'd0;
                point_count_o <= 6'd0;
                max_error_o   <= 12'd0;
                fit_valid_o   <= 1'b0;
                busy_o        <= 1'b0;
            end else begin
                if (point_valid_i && (state == S_IDLE)) begin
                    if (point_count < MAX_POINTS) begin
                        point_row[point_count] <= row_idx_i;
                        point_x[point_count]   <= x_i;
                    end
                    point_count <= point_count + 1'b1;
                    sum_x       <= sum_x + row_idx_i;
                    sum_y       <= sum_y + x_i;
                    sum_xx      <= sum_xx + row_idx_i * row_idx_i;
                    sum_xy      <= sum_xy + row_idx_i * x_i;
                    point_count_o <= point_count + 1'b1;
                end

                case (state)
                    S_IDLE: begin
                        if (frame_end_i) begin
                            den <= den_eval;
                            num <= num_eval;

                            if ((point_count < MIN_POINTS) ||
                                (point_count > MAX_POINTS) ||
                                (den_eval <= 0)) begin
                                max_error_o <= 12'd0;
                                fit_valid_o <= 1'b0;
                                done_o      <= 1'b1;
                                state       <= S_DONE;
                            end else begin
                                sign_a         <= num_eval[63];
                                div_a_dividend <= num_eval_abs[55:0] << 8;
                                div_a_divisor  <= den_eval[31:0];
                                div_a_start    <= 1'b1;
                                busy_o         <= 1'b1;
                                state          <= S_DIV_A;
                            end
                        end
                    end

                    S_DIV_A: begin
                        if (div_a_done) begin
                            if (sign_a)
                                a_q8_o <= -$signed(div_a_quotient[31:0]);
                            else
                                a_q8_o <= $signed(div_a_quotient[31:0]);

                            b_num <= b_num_next;
                            sign_b <= b_num_next[63];
                            div_b_dividend <= b_num_next_abs[55:0];
                            div_b_divisor <= {26'd0, point_count};
                            div_b_start   <= 1'b1;
                            state         <= S_CALC_B;
                        end
                    end

                    S_CALC_B: begin
                        // One wait cycle lets div_b_start propagate cleanly.
                        state <= S_DIV_B;
                    end

                    S_DIV_B: begin
                        if (div_b_done) begin
                            if (sign_b)
                                b_q8_o <= -$signed(div_b_quotient[31:0]);
                            else
                                b_q8_o <= $signed(div_b_quotient[31:0]);
                            resid_idx <= 6'd0;
                            max_error <= 12'd0;
                            state     <= S_RESID;
                        end
                    end

                    S_RESID: begin
                        if (resid_idx < point_count) begin
                            max_error <= max_error_next;
                            resid_idx <= resid_idx + 1'b1;
                        end else begin
                            max_error_o <= max_error;
                            fit_valid_o <= 1'b1;
                            busy_o      <= 1'b0;
                            done_o      <= 1'b1;
                            state       <= S_DONE;
                        end
                    end

                    S_DONE: begin
                        state <= S_IDLE;
                    end

                    default: state <= S_IDLE;
                endcase
            end
        end
    end

    // Keep the captured denominator in the waveform for debug.
    wire unused_den = (den != 0);
    wire unused_div_a_busy = div_a_busy;
    wire unused_div_b_busy = div_b_busy;

endmodule
