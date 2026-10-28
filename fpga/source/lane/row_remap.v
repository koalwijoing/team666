// ============================================================
// row_remap.v - 逐扫描行 IPM 最近邻重映射
// ============================================================
// 算法来源: ai/scripts/43_车道线检测_逐行重映射.py remap_row()
// 参数来源: per-row start/step Q16.16 calibration LUT
// 验收判据: tb_row_remap.v identity/scale/offset sequences
// 映射关系：
//   src_q16(dst) = src_start_q16_i + dst * src_step_q16_i
//   src_x = src_q16(dst) >> 16
//
// 输入源行按 x=0,1,2... 顺序提供；模块按需要请求源 bit。
// 输出目标行按 dst=0,1,2... 顺序逐拍输出。
// 不插值，不使用二维 warp。
// ============================================================
`timescale 1ns/1ps

module row_remap #(
    parameter integer WIDTH     = 1280,
    parameter integer X_WIDTH   = 11
) (
    input  wire                 clk,
    input  wire                 rst_n,

    input  wire                 start_i,
    input  wire [31:0]          src_start_q16_i,
    input  wire [31:0]          src_step_q16_i,

    output reg                  src_pop_o,
    input  wire                 src_valid_i,
    input  wire                 src_bit_i,

    output reg                  dst_valid_o,
    output reg                  dst_bit_o,
    output reg                  dst_last_o,
    output reg                  busy_o
);

    localparam [1:0] S_IDLE = 2'd0;
    localparam [1:0] S_RUN  = 2'd1;
    localparam [1:0] S_DONE = 2'd2;

    reg [1:0]         state;
    reg [31:0]        pos_q16;
    reg [31:0]        step_q16;
    reg [X_WIDTH:0]   src_idx;
    reg [X_WIDTH-1:0] dst_idx;
    reg               current_bit;
    reg               have_current;
    reg               pop_wait;

    wire [31:0] desired_src = pos_q16 >> 16;
    wire        desired_in_range = (desired_src < WIDTH);
    wire        need_source = desired_in_range &&
                              ((src_idx <= desired_src[ X_WIDTH-1:0]) ||
                               !have_current);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            pos_q16      <= 32'd0;
            step_q16     <= 32'd0;
            src_idx      <= {(X_WIDTH+1){1'b0}};
            dst_idx      <= {X_WIDTH{1'b0}};
            current_bit  <= 1'b0;
            have_current <= 1'b0;
            pop_wait     <= 1'b0;
            src_pop_o    <= 1'b0;
            dst_valid_o  <= 1'b0;
            dst_bit_o    <= 1'b0;
            dst_last_o   <= 1'b0;
            busy_o       <= 1'b0;
        end else begin
            dst_valid_o <= 1'b0;
            dst_last_o  <= 1'b0;
            src_pop_o   <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (start_i) begin
                        pos_q16      <= src_start_q16_i;
                        step_q16     <= src_step_q16_i;
                        src_idx      <= {(X_WIDTH+1){1'b0}};
                        dst_idx      <= {X_WIDTH{1'b0}};
                        current_bit  <= 1'b0;
                        have_current <= 1'b0;
                        pop_wait     <= 1'b0;
                        busy_o       <= 1'b1;
                        state        <= S_RUN;
                    end
                end

                S_RUN: begin
                    if (!desired_in_range) begin
                        dst_valid_o <= 1'b1;
                        dst_bit_o   <= 1'b0;
                        dst_last_o  <= (dst_idx == (WIDTH - 1));
                        pos_q16     <= pos_q16 + step_q16;
                        if (dst_idx == (WIDTH - 1))
                            state <= S_DONE;
                        else
                            dst_idx <= dst_idx + 1'b1;
                    end else if (need_source) begin
                        if (pop_wait) begin
                            pop_wait <= 1'b0;
                        end else if (src_valid_i) begin
                            current_bit  <= src_bit_i;
                            have_current <= 1'b1;
                            src_idx      <= src_idx + 1'b1;
                            src_pop_o    <= 1'b1;
                            pop_wait     <= 1'b1;
                        end
                    end else begin
                        dst_valid_o <= 1'b1;
                        dst_bit_o   <= current_bit;
                        dst_last_o  <= (dst_idx == (WIDTH - 1));
                        pos_q16     <= pos_q16 + step_q16;
                        if (dst_idx == (WIDTH - 1))
                            state <= S_DONE;
                        else
                            dst_idx <= dst_idx + 1'b1;
                    end
                end

                S_DONE: begin
                    busy_o <= 1'b0;
                    state  <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
