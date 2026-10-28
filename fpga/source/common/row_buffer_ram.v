// ============================================================
// row_buffer_ram.v - simple dual-port line-buffer RAM
// ============================================================
// 算法来源: Pango DRM line-buffer primitive wrapper
// 参数来源: ADDR_WIDTH/DATA_WIDTH from row buffer usage
// 验收判据: indirectly covered by blur/sobel/scan_row_capture tests
// ============================================================
`timescale 1ns/1ps

module row_buffer_ram #(
    parameter integer ADDR_WIDTH = 11,
    parameter integer DATA_WIDTH = 8
) (
    input  wire                    clk,
    input  wire                    wr_en,
    input  wire [ADDR_WIDTH-1:0]   wr_addr,
    input  wire [DATA_WIDTH-1:0]   wr_data,
    input  wire [ADDR_WIDTH-1:0]   rd_addr,
    output wire [DATA_WIDTH-1:0]   rd_data
);

`ifdef SIMULATION
    reg [DATA_WIDTH-1:0] mem [0:(1<<ADDR_WIDTH)-1];
    integer init_i;

    initial begin
        for (init_i = 0; init_i < (1<<ADDR_WIDTH); init_i = init_i + 1)
            mem[init_i] = {DATA_WIDTH{1'b0}};
    end

    always @(posedge clk) begin
        if (wr_en)
            mem[wr_addr] <= wr_data;
    end

    assign rd_data = mem[rd_addr];
`else
    wire [35:0] dob_bus;

    GTP_DRM36K_E1 #(
        .GRS_EN          ("FALSE"),
        .CSA_MASK        (0),
        .CSB_MASK        (0),
        .DATA_WIDTH_A    (DATA_WIDTH),
        .DATA_WIDTH_B    (DATA_WIDTH),
        .WRITE_MODE_A    ("NORMAL_WRITE"),
        .WRITE_MODE_B    ("NORMAL_WRITE"),
        .DOA_REG         (0),
        .DOB_REG         (0),
        .DOA_REG_CLKINV  (0),
        .DOB_REG_CLKINV  (0),
        .RST_TYPE        ("ASYNC"),
        .RAM_MODE        ("SIMPLE_DUAL_PORT"),
        .INIT_FILE       ("NONE"),
        .RAM_CASCADE     ("NONE"),
        .ECC_WRITE_EN    ("FALSE"),
        .ECC_READ_EN     ("FALSE"),
        .BLOCK_X         (0),
        .BLOCK_Y         (0),
        .RAM_ADDR_WIDTH  (ADDR_WIDTH),
        .RAM_DATA_WIDTH  (DATA_WIDTH),
        .INIT_FORMAT     ("BIN")
    ) u_ram (
        .DOA             (),
        .ADDRA           ({{(16-ADDR_WIDTH){1'b0}}, wr_addr}),
        .ADDRA_HOLD      (1'b0),
        .BWEA            (8'hff),
        .DIA             ({{(36-DATA_WIDTH){1'b0}}, wr_data}),
        .CSA             (3'b000),
        .WEA             (wr_en),
        .CLKA            (clk),
        .CEA             (1'b1),
        .ORCEA           (1'b0),
        .RSTA            (1'b0),
        .CINA            (1'b0),
        .COUTA           (),

        .DOB             (dob_bus),
        .ADDRB           ({{(16-ADDR_WIDTH){1'b0}}, rd_addr}),
        .ADDRB_HOLD      (1'b0),
        .BWEB            (4'b0000),
        .DIB             (36'b0),
        .CSB             (3'b000),
        .WEB             (1'b0),
        .CLKB            (clk),
        .CEB             (1'b1),
        .ORCEB           (1'b0),
        .RSTB            (1'b0),
        .CINB            (1'b0),
        .COUTB           (),

        .INJECT_SBITERR  (1'b0),
        .INJECT_DBITERR  (1'b0),
        .ECC_SBITERR     (),
        .ECC_DBITERR     (),
        .ECC_RDADDR      (),
        .ECC_PARITY      ()
    );

    assign rd_data = dob_bus[DATA_WIDTH-1:0];
`endif

endmodule
