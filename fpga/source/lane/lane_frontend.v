// ============================================================
// lane_frontend.v - RGB565 to binary edge stream
// ============================================================
// 算法来源: PC v3 gray -> blur -> Sobel-x -> threshold
// 参数来源: rgb coefficients, 3x3 kernel, threshold 42
// 验收判据: tb_lane_frontend.v 8x4 full-pixel reference compare
// Pipeline:
//   RGB565 -> gray -> blur3x3 -> sobel_x -> edge_threshold
//
// Output edge bits are aligned with delayed x/y/eol coordinates.
// ============================================================
`timescale 1ns/1ps

module lane_frontend #(
    parameter integer WIDTH    = 1280,
    parameter integer X_WIDTH  = 11,
    parameter integer Y_WIDTH  = 11
) (
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 frame_start_i,
    input  wire                 valid_i,
    input  wire                 eol_i,
    input  wire [15:0]          rgb565_i,
    input  wire [10:0]          threshold_i,

    output wire                 edge_valid_o,
    output wire                 edge_bit_o,
    output wire [X_WIDTH-1:0]   x_o,
    output wire [Y_WIDTH-1:0]   y_o,
    output wire                 eol_o
);

    wire gray_valid;
    wire [7:0] gray;
    wire blur_valid;
    wire [7:0] blur;
    wire sobel_valid;
    wire [10:0] grad;

    reg [X_WIDTH-1:0] x_count;
    reg [Y_WIDTH-1:0] y_count;

    reg [X_WIDTH-1:0] x_pipe [0:4];
    reg [Y_WIDTH-1:0] y_pipe [0:4];
    reg               eol_pipe [0:4];

    rgb565_to_gray u_gray (
        .clk(clk),
        .rst_n(rst_n),
        .valid_i(valid_i),
        .rgb565_i(rgb565_i),
        .valid_o(gray_valid),
        .gray_o(gray)
    );

    blur3x3 #(
        .WIDTH(WIDTH)
    ) u_blur (
        .clk(clk),
        .rst_n(rst_n),
        .frame_start_i(frame_start_i),
        .valid_i(gray_valid),
        .eol_i(eol_pipe[0]),
        .pixel_i(gray),
        .valid_o(blur_valid),
        .pixel_o(blur)
    );

    sobel_x #(
        .WIDTH(WIDTH)
    ) u_sobel (
        .clk(clk),
        .rst_n(rst_n),
        .frame_start_i(frame_start_i),
        .valid_i(blur_valid),
        .eol_i(eol_pipe[1]),
        .pixel_i(blur),
        .valid_o(sobel_valid),
        .grad_o(grad)
    );

    edge_threshold u_threshold (
        .clk(clk),
        .rst_n(rst_n),
        .valid_i(sobel_valid),
        .grad_i(grad),
        .threshold_i(threshold_i),
        .valid_o(edge_valid_o),
        .edge_o(edge_bit_o)
    );

    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_count <= {X_WIDTH{1'b0}};
            y_count <= {Y_WIDTH{1'b0}};
            for (k = 0; k <= 4; k = k + 1) begin
                x_pipe[k] <= {X_WIDTH{1'b0}};
                y_pipe[k] <= {Y_WIDTH{1'b0}};
                eol_pipe[k] <= 1'b0;
            end
        end else if (frame_start_i) begin
            x_count <= {X_WIDTH{1'b0}};
            y_count <= {Y_WIDTH{1'b0}};
            for (k = 0; k <= 4; k = k + 1) begin
                x_pipe[k] <= {X_WIDTH{1'b0}};
                y_pipe[k] <= {Y_WIDTH{1'b0}};
                eol_pipe[k] <= 1'b0;
            end
        end else begin
            for (k = 4; k > 0; k = k - 1) begin
                x_pipe[k] <= x_pipe[k-1];
                y_pipe[k] <= y_pipe[k-1];
                eol_pipe[k] <= eol_pipe[k-1];
            end

            // EOL arrives on the cycle AFTER the last active pixel (the
            // adapter registers it), i.e. during blanking where valid_i is
            // low. It must be handled outside the valid gate or it is
            // dropped: x_count then free-runs past WIDTH and y_count stays
            // 0, so no scan row ever matches and nothing is captured.
            if (eol_i) begin
                x_pipe[0] <= x_count;
                y_pipe[0] <= y_count;
                eol_pipe[0] <= 1'b1;
                x_count <= {X_WIDTH{1'b0}};
                y_count <= y_count + 1'b1;
            end else if (valid_i) begin
                x_pipe[0] <= x_count;
                y_pipe[0] <= y_count;
                eol_pipe[0] <= 1'b0;

                x_count <= x_count + 1'b1;
            end else begin
                x_pipe[0] <= {X_WIDTH{1'b0}};
                y_pipe[0] <= {Y_WIDTH{1'b0}};
                eol_pipe[0] <= 1'b0;
            end
        end
    end

    assign x_o = x_pipe[3];
    assign y_o = y_pipe[3];
    assign eol_o = eol_pipe[3];

endmodule
