// ============================================================
// blur3x3.v - streaming 3x3 mean filter
// ============================================================
// 算法来源: ai/scripts/43_车道线检测_逐行重映射.py cv2.blur
// 参数来源: 3x3 normalized kernel
// 验收判据: tb_blur3x3.v full-window pixel comparison
// Uses two line buffers and three horizontal delay taps.
// pixel_o = (sum3x3 + 4) / 9, matching normalized box blur.
// ============================================================
`timescale 1ns/1ps

module blur3x3 #(
    parameter integer WIDTH = 1280
) (
    input  wire             clk,
    input  wire             rst_n,
    input  wire             frame_start_i,
    input  wire             valid_i,
    input  wire             eol_i,
    input  wire [7:0]       pixel_i,
    output reg              valid_o,
    output reg  [7:0]       pixel_o
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

    wire [11:0] sum3x3 =
        {4'd0, top_d2} + {4'd0, top_d1} + {4'd0, top_x} +
        {4'd0, mid_d2} + {4'd0, mid_d1} + {4'd0, mid_x} +
        {4'd0, cur_d2} + {4'd0, cur_d1} + {4'd0, pixel_i};

    wire [7:0] blur_value = (sum3x3 + 12'd4) / 12'd9;

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
            pixel_o <= 8'd0;
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
                // Form the current 3x3 window from the registered taps.
                valid_o <= 1'b1;
                if ((row_count >= 10'd2) && (col_count >= 10'd2))
                    pixel_o <= blur_value;
                else
                    pixel_o <= 8'd0;

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
