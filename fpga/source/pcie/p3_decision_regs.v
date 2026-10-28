// ============================================================
// p3_decision_regs.v - P3 decision and key registers at BAR0+0x200
// ============================================================
`timescale 1ns/1ps

module p3_decision_regs #(
    parameter integer CLK_HZ         = 125_000_000,
    parameter integer DEBOUNCE_MS    = 20
) (
    input  wire                 clk,
    input  wire                 rst_n,

    input  wire                 key0_n,
    input  wire                 key1_n,

    output reg                  led_l,
    output reg                  led_r,
    output reg                  led_m,
    output reg                  alarm,
    output wire [15:0]          frame_echo,
    output wire [4:0]           key_events,

    input  wire [9:0]           wr_addr_i,
    input  wire [31:0]          wdata_i,
    input  wire                 wr_i,
    input  wire [9:0]           rd_addr_i,
    output reg  [31:0]          rdata_o,
    output reg  [127:0]         rdata_128_o
);

    localparam [9:0] A_CTRL   = 10'h200;
    localparam [9:0] A_STATUS = 10'h204;
    localparam [9:0] A_EVENT  = 10'h208;
    localparam [9:0] A_INJECT = 10'h20c;
    localparam integer DEBOUNCE_CYCLES =
        (CLK_HZ / 1000) * DEBOUNCE_MS;

    reg [15:0] frame_echo_r;
    reg [4:0]  key_event_r;

    wire key0_pressed;
    wire key1_pressed;
    wire key0_press;
    wire key1_press;

    wire [31:0] ctrl_word = {frame_echo_r, 12'd0, alarm, led_m, led_r, led_l};
    wire [31:0] status_word = {
        frame_echo_r, 3'd0, key_event_r, 2'd0,
        key1_pressed, key0_pressed, alarm, led_m, led_r, led_l
    };

    assign frame_echo = frame_echo_r;
    assign key_events = key_event_r;

    p3_key_debounce #(
        .DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)
    ) u_key0 (
        .key_n_async(key0_n),
        .clk(clk),
        .rst_n(rst_n),
        .pressed(key0_pressed),
        .press_pulse(key0_press)
    );

    p3_key_debounce #(
        .DEBOUNCE_CYCLES(DEBOUNCE_CYCLES)
    ) u_key1 (
        .key_n_async(key1_n),
        .clk(clk),
        .rst_n(rst_n),
        .pressed(key1_pressed),
        .press_pulse(key1_press)
    );

    always @* begin
        rdata_o = 32'd0;
        case (rd_addr_i)
            A_CTRL:   rdata_o = ctrl_word;
            A_STATUS: rdata_o = status_word;
            A_EVENT:  rdata_o = {27'd0, key_event_r};
            A_INJECT: rdata_o = 32'd0;
            default:  rdata_o = 32'd0;
        endcase
    end

    always @* begin
        rdata_128_o = {
            32'd0,
            {27'd0, key_event_r},
            status_word,
            ctrl_word
        };
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            led_l       <= 1'b0;
            led_r       <= 1'b0;
            led_m       <= 1'b1;
            alarm       <= 1'b0;
            frame_echo_r <= 16'd0;
            key_event_r <= 5'd0;
        end else begin
            if (key0_press)
                key_event_r[0] <= 1'b1;
            if (key1_press)
                key_event_r[1] <= 1'b1;

            if (wr_i) begin
                case (wr_addr_i)
                    A_CTRL: begin
                        led_l        <= wdata_i[0];
                        led_r        <= wdata_i[1];
                        led_m        <= wdata_i[2];
                        alarm        <= wdata_i[3];
                        frame_echo_r <= wdata_i[31:16];
                    end

                    A_EVENT: begin
                        key_event_r <= key_event_r & ~wdata_i[4:0];
                    end

                    A_INJECT: begin
                        key_event_r <= key_event_r | wdata_i[4:0];
                    end

                    default: begin
                    end
                endcase
            end
        end
    end

endmodule
