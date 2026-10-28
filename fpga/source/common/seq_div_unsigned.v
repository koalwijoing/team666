// ============================================================
// seq_div_unsigned.v - 56-bit / 32-bit sequential divider
// ============================================================
// 算法来源: restoring unsigned division
// 参数来源: 56-bit dividend / 32-bit divisor for Q8.8 fit
// 验收判据: indirectly covered by tb_line_fit.v exact fits
// Restoring divider. One quotient bit is produced per clock.
// The quotient is 56 bits wide so it can hold (48-bit numerator << 8).
// ============================================================
`timescale 1ns/1ps

module seq_div_unsigned (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start_i,
    input  wire [55:0] dividend_i,
    input  wire [31:0] divisor_i,
    output reg  [55:0] quotient_o,
    output reg  [31:0] remainder_o,
    output reg         busy_o,
    output reg         done_o
);

    reg [55:0] dividend_shift;
    reg [55:0] quotient_work;
    reg [32:0] remainder_work;
    reg [6:0]  bit_count;

    wire [32:0] shifted_remainder =
        {remainder_work[31:0], dividend_shift[55]};
    wire [32:0] divisor_ext = {1'b0, divisor_i};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dividend_shift  <= 56'd0;
            quotient_work   <= 56'd0;
            remainder_work  <= 33'd0;
            bit_count       <= 7'd0;
            quotient_o      <= 56'd0;
            remainder_o     <= 32'd0;
            busy_o          <= 1'b0;
            done_o          <= 1'b0;
        end else begin
            done_o <= 1'b0;

            if (start_i && !busy_o) begin
                if (divisor_i == 32'd0) begin
                    quotient_o  <= 56'd0;
                    remainder_o <= 32'd0;
                    busy_o      <= 1'b0;
                    done_o      <= 1'b1;
                end else begin
                    dividend_shift <= dividend_i;
                    quotient_work  <= 56'd0;
                    remainder_work <= 33'd0;
                    bit_count      <= 7'd56;
                    busy_o         <= 1'b1;
                end
            end else if (busy_o) begin
                if (bit_count == 7'd0) begin
                    quotient_o  <= quotient_work;
                    remainder_o <= remainder_work[31:0];
                    busy_o      <= 1'b0;
                    done_o      <= 1'b1;
                end else begin
                    if (shifted_remainder >= divisor_ext) begin
                        remainder_work <= shifted_remainder - divisor_ext;
                        quotient_work  <= {quotient_work[54:0], 1'b1};
                    end else begin
                        remainder_work <= shifted_remainder;
                        quotient_work  <= {quotient_work[54:0], 1'b0};
                    end

                    dividend_shift <= {dividend_shift[54:0], 1'b0};
                    bit_count      <= bit_count - 1'b1;
                end
            end
        end
    end

endmodule
