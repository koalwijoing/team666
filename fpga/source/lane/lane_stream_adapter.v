// ============================================================
// lane_stream_adapter.v - DVP stream to lane_top control signals
// ============================================================
// Converts a cmos_8_16bit-style DE/VS stream into the frame
// controls expected by lane_top. The RGB565 payload is passed
// through unchanged.
// ============================================================
`timescale 1ns/1ps

module lane_stream_adapter #(
    parameter integer H_ACTIVE = 1280,
    parameter integer V_ACTIVE = 720,
    parameter integer X_WIDTH  = 11,
    parameter integer Y_WIDTH  = 11
) (
    input  wire                 clk,
    input  wire                 rst_n,

    input  wire                 vsync_i,
    input  wire                 de_i,
    input  wire [15:0]          rgb565_i,

    output reg                  frame_start_o,
    output wire                 valid_o,
    output wire [15:0]          rgb565_o,
    output wire                 eol_o,
    output reg                  frame_done_o,
    output reg  [31:0]          frame_id_o
);

    reg                     vsync_meta;
    reg                     vsync_sync;
    reg                     vsync_sync_d;
    reg [X_WIDTH-1:0]       x_cnt;
    reg [Y_WIDTH-1:0]       row_cnt;
    reg                     last_row_eol;

    // Rising edge only: exactly one frame start per VSYNC pulse. The
    // any-edge form pulsed twice per frame and reset the scan counters
    // mid-stream (frame_id also advanced twice per frame).
    wire frame_start_pulse = vsync_sync && !vsync_sync_d;
    wire last_pixel        = de_i && (x_cnt == H_ACTIVE - 1);
    wire last_row          = last_pixel && (row_cnt == V_ACTIVE - 1);

    assign valid_o  = de_i;
    assign rgb565_o = rgb565_i;
    // eol must assert DURING the last active pixel, not one cycle later
    // during blanking: downstream stages (blur/sobel/frontend coordinate
    // counters) only sample eol_i while valid_i is high and drop it
    // otherwise, which free-ran their counters and killed all captures.
    assign eol_o    = last_pixel;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vsync_meta    <= 1'b0;
            vsync_sync    <= 1'b0;
            vsync_sync_d  <= 1'b0;
            x_cnt         <= {X_WIDTH{1'b0}};
            row_cnt       <= {Y_WIDTH{1'b0}};
            last_row_eol  <= 1'b0;
            frame_start_o <= 1'b0;
            frame_done_o  <= 1'b0;
            frame_id_o    <= 32'd0;
        end else begin
            vsync_meta   <= vsync_i;
            vsync_sync   <= vsync_meta;
            vsync_sync_d <= vsync_sync;

            frame_start_o <= 1'b0;
            frame_done_o  <= 1'b0;

            if (frame_start_pulse) begin
                frame_start_o <= 1'b1;
                frame_id_o    <= frame_id_o + 1'b1;
                x_cnt         <= {X_WIDTH{1'b0}};
                row_cnt       <= {Y_WIDTH{1'b0}};
                last_row_eol  <= 1'b0;
            end else if (de_i) begin
                if (last_pixel) begin
                    x_cnt <= {X_WIDTH{1'b0}};
                    if (last_row)
                        last_row_eol <= 1'b1;
                    else
                        row_cnt <= row_cnt + 1'b1;
                end else begin
                    x_cnt <= x_cnt + 1'b1;
                end
            end

            if (last_row_eol) begin
                frame_done_o <= 1'b1;
                last_row_eol <= 1'b0;
                row_cnt      <= {Y_WIDTH{1'b0}};
            end
        end
    end

endmodule
