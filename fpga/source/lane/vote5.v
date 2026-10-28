// ============================================================
// vote5.v - 5 帧多数表决
// ============================================================
// 算法来源: ai/scripts/43_车道线检测_逐行重映射.py vote history
// 参数来源: fpga/pl/vectors/README.md, VOTE_N=5
// 验收判据: tb_vote5.v majority/tie cases + tb_golden_l1.v frame4
// 最新状态进入 history[0]。始终选择 5 帧窗口内出现次数最多的状态。
// PC 原型的 max(set(...)) 在平票时受哈希随机化影响，已改为确定性安全规则：
// SOLID(2) > NONE(0) > DASHED(1)。不确定时绝不授予变道许可。
// ============================================================
`timescale 1ns/1ps

module vote5 (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       valid_i,
    input  wire [1:0] state_i,
    output reg  [1:0] state_o
);

    reg [1:0] h0;
    reg [1:0] h1;
    reg [1:0] h2;
    reg [1:0] h3;
    reg [1:0] h4;
    wire [2:0] cnt0 = (h0 == 2'd0) + (h1 == 2'd0) + (h2 == 2'd0) +
                      (h3 == 2'd0) + (state_i == 2'd0);
    wire [2:0] cnt1 = (h0 == 2'd1) + (h1 == 2'd1) + (h2 == 2'd1) +
                      (h3 == 2'd1) + (state_i == 2'd1);
    wire [2:0] cnt2 = (h0 == 2'd2) + (h1 == 2'd2) + (h2 == 2'd2) +
                      (h3 == 2'd2) + (state_i == 2'd2);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            h0 <= 2'd3;
            h1 <= 2'd3;
            h2 <= 2'd3;
            h3 <= 2'd3;
            h4 <= 2'd3;
            state_o <= 2'd0;
        end else if (valid_i) begin
            h0 <= state_i;
            h1 <= h0;
            h2 <= h1;
            h3 <= h2;
            h4 <= h3;

            if ((cnt2 >= cnt0) && (cnt2 >= cnt1)) begin
                state_o <= 2'd2;
            end else if (cnt0 >= cnt1) begin
                state_o <= 2'd0;
            end else begin
                state_o <= 2'd1;
            end
        end
    end

endmodule
