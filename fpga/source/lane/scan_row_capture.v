// ============================================================
// scan_row_capture.v - selected scan-row edge storage
// ============================================================
// 算法来源: ai/scripts/43_车道线检测_逐行重映射.py scan row collection
// 参数来源: WIDTH=1280, SCAN_ROWS=21, Y0=432, STEP=12
// 验收判据: tb_lane_rows.v RAM content probe + tb_lane_sim.v tracking
// 16 columns pack into one word. Columns arrive strictly left-to-right
// within a row, so a one-entry write-first accumulator produces each word
// without any read-modify-write (an earlier RMW through the shared read
// port never captured anything; a 1-bit-wide DRAM mapping then read back
// as constant zero on silicon, so this keeps a plain 16-bit memory that
// synthesizes to the well-proven block/distributed RAM mappings).
`timescale 1ns/1ps

module scan_row_capture #(
    parameter integer WIDTH      = 1280,
    parameter integer X_WIDTH    = 11,
    parameter integer SCAN_ROWS  = 21
) (
    input  wire                  clk,
    input  wire                  rst_n,

    input  wire                  capture_valid_i,
    input  wire [4:0]            capture_row_i,
    input  wire [X_WIDTH-1:0]    capture_x_i,
    input  wire                  capture_bit_i,

    input  wire [4:0]            rd_row_i,
    input  wire [X_WIDTH-1:0]    rd_x_i,
    output wire                  rd_bit_o
);

    wire [10:0] capture_x_ext = {{(11-X_WIDTH){1'b0}}, capture_x_i};
    wire [10:0] rd_x_ext = {{(11-X_WIDTH){1'b0}}, rd_x_i};

    localparam integer WORDS_PER_ROW = WIDTH / 16;  // 80

    function [14:0] row_offset;
        input [4:0] row;
        begin
            row_offset = row * WORDS_PER_ROW;
        end
    endfunction

    wire [6:0]  cap_word_idx = capture_x_ext[10:4];
    wire [6:0]  rd_word_idx  = rd_x_ext[10:4];
    wire [14:0] wr_addr = row_offset(capture_row_i) + cap_word_idx;
    wire [14:0] rd_addr = row_offset(rd_row_i) + rd_word_idx;

    reg  [15:0] mem [0:SCAN_ROWS*WORDS_PER_ROW-1];

    // Write-first accumulator: a new word address starts a fresh zero word;
    // consecutive writes to the same word OR in their bit. Every write
    // stores the accumulated-so-far value, so the last write to a word
    // leaves the complete word behind (no read path involved).
    reg [15:0] acc;
    reg [14:0] acc_addr;
    reg        acc_valid;

    wire [15:0] bit_mask = 16'b1 << capture_x_ext[3:0];
    wire [15:0] acc_base =
        (acc_valid && (acc_addr == wr_addr)) ? acc : 16'd0;
    wire [15:0] word_value = capture_bit_i
                             ? (acc_base | bit_mask)
                             : (acc_base & ~bit_mask);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc       <= 16'd0;
            acc_addr  <= 15'd0;
            acc_valid <= 1'b0;
        end else if (capture_valid_i) begin
            mem[wr_addr] <= word_value;
            acc          <= word_value;
            acc_addr     <= wr_addr;
            acc_valid    <= 1'b1;
        end
    end

    reg [15:0] rd_word;
    always @(*) rd_word = mem[rd_addr];
    assign rd_bit_o = rd_word[rd_x_ext[3:0]];

endmodule
