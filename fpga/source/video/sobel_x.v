// ============================================================
// sobel_x.v - streaming horizontal Sobel gradient
// ============================================================
// 算法来源: ai/scripts/43_车道线检测_逐行重映射.py cv2.Sobel
// 参数来源: ksize=3, horizontal derivative, absolute magnitude
// 验收判据: tb_sobel_x.v full-window pixel comparison
// Gx = right_column*4 - left_column*4
// Output is the absolute magnitude.
// ============================================================
`timescale 1ns/1ps

module sobel_x #(
    parameter integer WIDTH = 1280
) (
    input  wire             clk,
    input  wire             rst_n,
    input  wire             frame_start_i,
    input  wire             valid_i,
    input  wire             eol_i,
    input  wire [7:0]       pixel_i,
    output reg              valid_o,
    output reg  [10:0]      grad_o
);

    wire [7:0] line_m1_rd;
    wire [7:0] line_m2_rd;

    reg [10:0] col_count;
    reg [9:0] row_count;

    reg [7:0] cur_d1;
    reg [7:0] cur_d2;
    reg [7:0] mid_d1;
    reg [7:0] mid_d2;
    reg [7:0] top_d1;
    reg [7:0] top_d2;

    wire [7:0] top_x = line_m2_rd;
    wire [7:0] mid_x = line_m1_rd;

    row_buffer_ram #(
        .ADDR_WIDTH(11),
        .DATA_WIDTH(8)
    ) u_line_m1 (
        .clk(clk),
        .wr_en(valid_i),
        .wr_addr(col_count),
        .wr_data(pixel_i),
        .rd_addr(col_count),
        .rd_data(line_m1_rd)
    );

    row_buffer_ram #(
        .ADDR_WIDTH(11),
        .DATA_WIDTH(8)
    ) u_line_m2 (
        .clk(clk),
        .wr_en(valid_i),
        .wr_addr(col_count),
        .wr_data(line_m1_rd),
        .rd_addr(col_count),
        .rd_data(line_m2_rd)
    );

    wire signed [11:0] gx =
        {4'd0, top_x}   + ({1'b0, mid_x, 1'b0})   + {4'd0, pixel_i} -
        {4'd0, top_d2}  - ({1'b0, mid_d2, 1'b0})  - {4'd0, cur_d2};

    wire signed [11:0] gx_abs_s = (gx < 0) ? -gx : gx;
    wire [10:0] gx_abs = gx_abs_s[10:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            col_count <= 10'd0;
            row_count <= 10'd0;
            cur_d1 <= 8'd0;
            cur_d2 <= 8'd0;
            mid_d1 <= 8'd0;
            mid_d2 <= 8'd0;
            top_d1 <= 8'd0;
            top_d2 <= 8'd0;
            valid_o <= 1'b0;
            grad_o <= 11'd0;
        end else begin
            valid_o <= 1'b0;

            if (frame_start_i) begin
                col_count <= 10'd0;
                row_count <= 10'd0;
                cur_d1 <= 8'd0;
                cur_d2 <= 8'd0;
                mid_d1 <= 8'd0;
                mid_d2 <= 8'd0;
                top_d1 <= 8'd0;
                top_d2 <= 8'd0;
            end else if (valid_i) begin
                valid_o <= 1'b1;
                if ((row_count >= 10'd2) && (col_count >= 10'd2))
                    grad_o <= gx_abs;
                else
                    grad_o <= 11'd0;

                cur_d2 <= cur_d1;
                cur_d1 <= pixel_i;
                mid_d2 <= mid_d1;
                mid_d1 <= mid_x;
                top_d2 <= top_d1;
                top_d1 <= top_x;

                if (eol_i) begin
                    col_count <= 10'd0;
                    row_count <= row_count + 1'b1;
                    cur_d1 <= 8'd0;
                    cur_d2 <= 8'd0;
                    mid_d1 <= 8'd0;
                    mid_d2 <= 8'd0;
                    top_d1 <= 8'd0;
                    top_d2 <= 8'd0;
                end else begin
                    col_count <= col_count + 1'b1;
                end
            end
        end
    end

endmodule
