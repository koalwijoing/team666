// ============================================================
// rgb565_to_gray.v - RGB565 to 8-bit luma
// ============================================================
// 算法来源: OpenCV COLOR_RGB2GRAY fixed-point coefficients
// 参数来源: 19595/38470/7471, rounding +32768, shift 16
// 验收判据: tb_rgb565_to_gray.v black/white/R/G/B case
// Y = (19595*R8 + 38470*G8 + 7471*B8 + 32768) >> 16
// RGB565 channels are expanded to 8 bits by bit replication.
// ============================================================
`timescale 1ns/1ps

module rgb565_to_gray (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        valid_i,
    input  wire [15:0] rgb565_i,
    output reg         valid_o,
    output reg  [7:0]  gray_o
);

    wire [4:0] r5 = rgb565_i[15:11];
    wire [5:0] g6 = rgb565_i[10:5];
    wire [4:0] b5 = rgb565_i[4:0];

    wire [7:0] r8 = {r5, r5[4:2]};
    wire [7:0] g8 = {g6, g6[5:4]};
    wire [7:0] b8 = {b5, b5[4:2]};

    wire [25:0] y_sum = 26'd19595 * r8 +
                        26'd38470 * g8 +
                        26'd7471  * b8 +
                        26'd32768;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_o <= 1'b0;
            gray_o  <= 8'd0;
        end else begin
            valid_o <= valid_i;
            if (valid_i)
                gray_o <= y_sum[23:16];
        end
    end

endmodule
