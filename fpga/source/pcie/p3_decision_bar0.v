// ============================================================
// p3_decision_bar0.v - BAR0 wrapper for the P3 decision registers
// ============================================================
`timescale 1ns/1ps

module p3_decision_bar0 #(
    parameter integer CLK_HZ      = 125_000_000,
    parameter integer DEBOUNCE_MS = 20
) (
    input  wire                 clk,
    input  wire                 rst_n,

    input  wire                 key0_n,
    input  wire                 key1_n,

    output wire                 led_l_o,
    output wire                 led_r_o,
    output wire                 led_m_o,
    output wire                 alarm_o,
    output wire [15:0]          frame_echo_o,
    output wire [4:0]           key_events_o,

    input  wire                 bar0_wr_en_i,
    input  wire [8:0]           bar0_wr_addr_i,
    input  wire [127:0]         bar0_wr_data_i,
    input  wire [15:0]          bar0_wr_be_i,
    input  wire [8:0]           bar0_rd_addr_i,

    output wire                 write_hit_o,
    output wire                 read_hit_o,
    output wire [127:0]         read_data_o
);

    localparam [8:0] DECISION_WORD  = 9'h20;
    localparam [8:0] READ_PIPE_WORD = 9'h21;

    wire write_hit_word = (bar0_wr_addr_i == DECISION_WORD);
    wire read_pipe_hit  = (bar0_rd_addr_i == READ_PIPE_WORD);
    wire [9:0] write_word_addr = 10'h200;

    reg [1:0] write_lane;
    reg [31:0] write_data;
    wire write_hit;
    wire read_hit;
    wire [127:0] decision_rdata;

    assign write_hit = bar0_wr_en_i && write_hit_word &&
                       (bar0_wr_be_i != 16'd0);
    assign read_hit  = read_pipe_hit;

    always @* begin
        if (bar0_wr_be_i[3:0] != 4'd0)
            write_lane = 2'd0;
        else if (bar0_wr_be_i[7:4] != 4'd0)
            write_lane = 2'd1;
        else if (bar0_wr_be_i[11:8] != 4'd0)
            write_lane = 2'd2;
        else
            write_lane = 2'd3;
    end

    always @* begin
        case (write_lane)
            2'd0:    write_data = bar0_wr_data_i[31:0];
            2'd1:    write_data = bar0_wr_data_i[63:32];
            2'd2:    write_data = bar0_wr_data_i[95:64];
            default: write_data = bar0_wr_data_i[127:96];
        endcase
    end

    p3_decision_regs #(
        .CLK_HZ(CLK_HZ),
        .DEBOUNCE_MS(DEBOUNCE_MS)
    ) u_regs (
        .clk(clk),
        .rst_n(rst_n),
        .key0_n(key0_n),
        .key1_n(key1_n),
        .led_l(led_l_o),
        .led_r(led_r_o),
        .led_m(led_m_o),
        .alarm(alarm_o),
        .frame_echo(frame_echo_o),
        .key_events(key_events_o),
        .wr_addr_i(write_word_addr + {6'b0, write_lane, 2'b00}),
        .wdata_i(write_data),
        .wr_i(write_hit),
        .rd_addr_i(10'h200),
        .rdata_o(),
        .rdata_128_o(decision_rdata)
    );

    assign write_hit_o = write_hit;
    assign read_hit_o  = read_hit;
    assign read_data_o = read_hit ? decision_rdata : 128'd0;

endmodule
