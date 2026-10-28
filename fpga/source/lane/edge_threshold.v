// ============================================================
// edge_threshold.v - Sobel magnitude threshold
// ============================================================
// 算法来源: ai/scripts/43_车道线检测_逐行重映射.py edge mask
// 参数来源: threshold default 42, runtime port
// 验收判据: tb_edge_threshold.v below/equal/above boundary
// Matches the PC prototype rule: edge = (gradient > threshold).
// ============================================================
`timescale 1ns/1ps

module edge_threshold (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        valid_i,
    input  wire [10:0] grad_i,
    input  wire [10:0] threshold_i,
    output reg         valid_o,
    output reg         edge_o
);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_o <= 1'b0;
            edge_o  <= 1'b0;
        end else begin
            valid_o <= valid_i;
            edge_o  <= valid_i && (grad_i > threshold_i);
        end
    end

endmodule
