// ============================================================
// p3_osd_font_rom.v - 8x16 glyph decoder for the P3 OSD overlay
// ============================================================
`timescale 1ns/1ps

module p3_osd_font_rom #(
    parameter INIT_FILE = "p3_osd_font.mem"
) (
    input  wire         clk,
    input  wire [8:0]   addr,
    output reg  [7:0]   dout
);

    function [7:0] font_row;
        input [4:0] glyph_id;
        input [3:0] glyph_row;
        reg   [2:0] source_row;
        reg   [4:0] pattern;
        begin
            font_row = 8'h00;
            if (glyph_row >= 4'd1 && glyph_row <= 4'd14) begin
                source_row = (glyph_row - 4'd1) >> 1;
                pattern = 5'b00000;

                case (glyph_id)
                    5'd1: case (source_row) // A
                        3'd0: pattern = 5'b01110;
                        3'd1: pattern = 5'b10001;
                        3'd2: pattern = 5'b10001;
                        3'd3: pattern = 5'b11111;
                        3'd4: pattern = 5'b10001;
                        3'd5: pattern = 5'b10001;
                        3'd6: pattern = 5'b10001;
                    endcase
                    5'd2: case (source_row) // E
                        3'd0: pattern = 5'b11111;
                        3'd1: pattern = 5'b10000;
                        3'd2: pattern = 5'b10000;
                        3'd3: pattern = 5'b11110;
                        3'd4: pattern = 5'b10000;
                        3'd5: pattern = 5'b10000;
                        3'd6: pattern = 5'b11111;
                    endcase
                    5'd3: case (source_row) // F
                        3'd0: pattern = 5'b11111;
                        3'd1: pattern = 5'b10000;
                        3'd2: pattern = 5'b10000;
                        3'd3: pattern = 5'b11110;
                        3'd4: pattern = 5'b10000;
                        3'd5: pattern = 5'b10000;
                        3'd6: pattern = 5'b10000;
                    endcase
                    5'd4: case (source_row) // K
                        3'd0: pattern = 5'b10001;
                        3'd1: pattern = 5'b10010;
                        3'd2: pattern = 5'b10100;
                        3'd3: pattern = 5'b11000;
                        3'd4: pattern = 5'b10100;
                        3'd5: pattern = 5'b10010;
                        3'd6: pattern = 5'b10001;
                    endcase
                    5'd5: case (source_row) // L
                        3'd0: pattern = 5'b10000;
                        3'd1: pattern = 5'b10000;
                        3'd2: pattern = 5'b10000;
                        3'd3: pattern = 5'b10000;
                        3'd4: pattern = 5'b10000;
                        3'd5: pattern = 5'b10000;
                        3'd6: pattern = 5'b11111;
                    endcase
                    5'd6: case (source_row) // M
                        3'd0: pattern = 5'b10001;
                        3'd1: pattern = 5'b11011;
                        3'd2: pattern = 5'b10101;
                        3'd3: pattern = 5'b10101;
                        3'd4: pattern = 5'b10001;
                        3'd5: pattern = 5'b10001;
                        3'd6: pattern = 5'b10001;
                    endcase
                    5'd7: case (source_row) // R
                        3'd0: pattern = 5'b11110;
                        3'd1: pattern = 5'b10001;
                        3'd2: pattern = 5'b10001;
                        3'd3: pattern = 5'b11110;
                        3'd4: pattern = 5'b10100;
                        3'd5: pattern = 5'b10010;
                        3'd6: pattern = 5'b10001;
                    endcase
                    5'd8: case (source_row) // Y
                        3'd0: pattern = 5'b10001;
                        3'd1: pattern = 5'b10001;
                        3'd2: pattern = 5'b01010;
                        3'd3: pattern = 5'b00100;
                        3'd4: pattern = 5'b00100;
                        3'd5: pattern = 5'b00100;
                        3'd6: pattern = 5'b00100;
                    endcase
                    5'd9: case (source_row) // 0
                        3'd0: pattern = 5'b01110;
                        3'd1: pattern = 5'b10001;
                        3'd2: pattern = 5'b10011;
                        3'd3: pattern = 5'b10101;
                        3'd4: pattern = 5'b11001;
                        3'd5: pattern = 5'b10001;
                        3'd6: pattern = 5'b01110;
                    endcase
                    5'd10: case (source_row) // 1
                        3'd0: pattern = 5'b00100;
                        3'd1: pattern = 5'b01100;
                        3'd2: pattern = 5'b00100;
                        3'd3: pattern = 5'b00100;
                        3'd4: pattern = 5'b00100;
                        3'd5: pattern = 5'b00100;
                        3'd6: pattern = 5'b01110;
                    endcase
                    5'd11: case (source_row) // 2
                        3'd0: pattern = 5'b01110;
                        3'd1: pattern = 5'b10001;
                        3'd2: pattern = 5'b00001;
                        3'd3: pattern = 5'b00010;
                        3'd4: pattern = 5'b00100;
                        3'd5: pattern = 5'b01000;
                        3'd6: pattern = 5'b11111;
                    endcase
                    5'd12: case (source_row) // 3
                        3'd0: pattern = 5'b11110;
                        3'd1: pattern = 5'b00001;
                        3'd2: pattern = 5'b00001;
                        3'd3: pattern = 5'b01110;
                        3'd4: pattern = 5'b00001;
                        3'd5: pattern = 5'b00001;
                        3'd6: pattern = 5'b11110;
                    endcase
                    5'd13: case (source_row) // 4
                        3'd0: pattern = 5'b00010;
                        3'd1: pattern = 5'b00110;
                        3'd2: pattern = 5'b01010;
                        3'd3: pattern = 5'b10010;
                        3'd4: pattern = 5'b11111;
                        3'd5: pattern = 5'b00010;
                        3'd6: pattern = 5'b00010;
                    endcase
                    5'd14: case (source_row) // 5
                        3'd0: pattern = 5'b11111;
                        3'd1: pattern = 5'b10000;
                        3'd2: pattern = 5'b10000;
                        3'd3: pattern = 5'b11110;
                        3'd4: pattern = 5'b00001;
                        3'd5: pattern = 5'b00001;
                        3'd6: pattern = 5'b11110;
                    endcase
                    default: pattern = 5'b00000;
                endcase

                font_row = {1'b0, pattern, 2'b00};
            end
        end
    endfunction

    reg [7:0] font_line;

    always @* begin
        font_line = font_row(addr[8:4], addr[3:0]);
    end

    always @(posedge clk) begin
        dout <= font_line;
    end

endmodule
