// ============================================================
// p3_osd_overlay.v - decision status overlay for the 640x360 stream
// ============================================================
`timescale 1ns/1ps

module p3_osd_overlay #(
    parameter INIT_FILE = "p3_osd_font.mem",
    parameter OSD_ENABLE = 1'b0
) (
    input  wire         clk_pixel,
    input  wire         rst_n,

    input  wire         vs_i,
    input  wire         valid_i,
    input  wire [15:0]  rgb565_i,

    input  wire         led_l_i,
    input  wire         led_m_i,
    input  wire         led_r_i,
    input  wire         alarm_i,

    output wire         valid_o,
    output wire [15:0]  rgb565_o
);

    localparam [15:0] COLOR_BLACK = 16'h0000;
    localparam [15:0] COLOR_GREEN = 16'h07e0;
    localparam [15:0] COLOR_YELLOW = 16'hffe0;
    localparam [15:0] COLOR_RED   = 16'hf800;
    localparam [15:0] COLOR_WHITE = 16'hffff;
    localparam [15:0] COLOR_DIM   = 16'h7bef;

    localparam [4:0] GLYPH_SPACE = 5'd0;
    localparam [4:0] GLYPH_A     = 5'd1;
    localparam [4:0] GLYPH_L     = 5'd5;
    localparam [4:0] GLYPH_M     = 5'd6;
    localparam [4:0] GLYPH_R     = 5'd7;

    reg [3:0] status_meta;
    reg [3:0] status_sync;
    wire [3:0] status_bits = status_sync;

    reg        vs_d;
    reg [9:0]  x;
    reg [8:0]  y;
    reg        osd_cell;
    reg [4:0]  glyph_id;
    reg [3:0]  glyph_row;
    reg [2:0]  glyph_col;
    reg [15:0] fg_color;
    reg [15:0] bg_color;

    wire       vs_pos = vs_i && !vs_d;
    wire       status_l = status_bits[0];
    wire       status_m = status_bits[1];
    wire       status_r = status_bits[2];
    wire       status_alarm = status_bits[3];

    always @(posedge clk_pixel or negedge rst_n) begin
        if (!rst_n) begin
            status_meta <= 4'b0;
            status_sync <= 4'b0;
        end else begin
            status_meta <= {alarm_i, led_r_i, led_m_i, led_l_i};
            status_sync <= status_meta;
        end
    end

    always @(posedge clk_pixel or negedge rst_n) begin
        if (!rst_n) begin
            vs_d <= 1'b0;
            x <= 10'd0;
            y <= 9'd0;
        end else begin
            vs_d <= vs_i;

            if (vs_pos) begin
                x <= 10'd0;
                y <= 9'd0;
            end else if (valid_i) begin
                if (x == 10'd639) begin
                    x <= 10'd0;
                    if (y == 9'd359)
                        y <= 9'd0;
                    else
                        y <= y + 9'd1;
                end else begin
                    x <= x + 10'd1;
                end
            end
        end
    end

    always @* begin
        osd_cell   = 1'b0;
        glyph_id   = GLYPH_SPACE;
        glyph_row  = 4'd0;
        glyph_col  = 3'd0;
        fg_color   = COLOR_DIM;
        bg_color   = COLOR_BLACK;

        if (OSD_ENABLE && y >= 9'd8 && y < 9'd24) begin
            if (x[9:4] == 6'd0 && x[3] == 1'b1) begin
                osd_cell  = x[3];
                glyph_id  = GLYPH_L;
                glyph_row = y[3:0] - 4'd8;
                glyph_col = x - 10'd8;
                fg_color  = status_l ? COLOR_GREEN : COLOR_DIM;
            end else if (x[9:5] == 5'd0 && x[4:3] == 2'b11) begin
                osd_cell  = 1'b1;
                glyph_id  = GLYPH_M;
                glyph_row = y[3:0] - 4'd8;
                glyph_col = x - 10'd24;
                fg_color  = status_m ? COLOR_YELLOW : COLOR_DIM;
            end else if (x[9:6] == 4'd0 && x[5:3] == 3'd5) begin
                osd_cell  = 1'b1;
                glyph_id  = GLYPH_R;
                glyph_row = y[3:0] - 4'd8;
                glyph_col = x - 10'd40;
                fg_color  = status_r ? COLOR_GREEN : COLOR_DIM;
            end
        end else if (OSD_ENABLE && y >= 9'd30 && y < 9'd46 &&
                     x[9:6] == 4'd0 &&
                     (x[5:3] == 3'd1 || x[5:3] == 3'd2 ||
                      x[5:3] == 3'd3 || x[5:3] == 3'd4 ||
                      x[5:3] == 3'd5)) begin
            osd_cell  = 1'b1;
            glyph_row = y[3:0] - 4'd14;
            glyph_col = x - 10'd8;
            case ((x - 10'd8) >> 3)
                3'd0: glyph_id = GLYPH_A;
                3'd1: glyph_id = GLYPH_L;
                3'd2: glyph_id = GLYPH_A;
                3'd3: glyph_id = GLYPH_R;
                3'd4: glyph_id = GLYPH_M;
                default: glyph_id = GLYPH_SPACE;
            endcase
            fg_color = status_alarm ? COLOR_WHITE : COLOR_DIM;
            bg_color = status_alarm ? COLOR_RED : COLOR_BLACK;
        end
    end

    wire [8:0] font_addr = {glyph_id, glyph_row};
    wire [7:0] font_data;
    p3_osd_font_rom #(
        .INIT_FILE(INIT_FILE)
    ) u_font_rom (
        .clk(clk_pixel),
        .addr(font_addr),
        .dout(font_data)
    );

    reg        info_cell_a;
    reg        info_cell_b;
    reg [2:0]  info_col_a;
    reg [2:0]  info_col_b;
    reg [15:0] info_fg_a;
    reg [15:0] info_fg_b;
    reg [15:0] info_bg_a;
    reg [15:0] info_bg_b;
    reg [15:0] image_a;
    reg [15:0] image_b;
    reg        valid_a;
    reg        valid_b;

    always @(posedge clk_pixel or negedge rst_n) begin
        if (!rst_n) begin
            info_cell_a <= 1'b0;
            info_cell_b <= 1'b0;
            info_col_a  <= 3'd0;
            info_col_b  <= 3'd0;
            info_fg_a   <= COLOR_DIM;
            info_fg_b   <= COLOR_DIM;
            info_bg_a   <= COLOR_BLACK;
            info_bg_b   <= COLOR_BLACK;
            image_a     <= 16'd0;
            image_b     <= 16'd0;
            valid_a     <= 1'b0;
            valid_b     <= 1'b0;
        end else begin
            info_cell_a <= osd_cell && valid_i;
            info_col_a  <= glyph_col;
            info_fg_a   <= fg_color;
            info_bg_a   <= bg_color;
            image_a     <= rgb565_i;
            valid_a     <= valid_i;

            info_cell_b <= info_cell_a;
            info_col_b  <= info_col_a;
            info_fg_b   <= info_fg_a;
            info_bg_b   <= info_bg_a;
            image_b     <= image_a;
            valid_b     <= valid_a;
        end
    end

    wire [2:0] font_bit_sel = 3'd7 - info_col_b;
    wire       font_pixel = font_data[font_bit_sel];
    wire [15:0] overlay_pixel = font_pixel ? info_fg_b : info_bg_b;

    assign valid_o  = valid_b;
    assign rgb565_o = valid_b ? (info_cell_b ? overlay_pixel : image_b) : image_b;

endmodule
