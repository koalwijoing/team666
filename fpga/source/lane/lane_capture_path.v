// ============================================================
// lane_capture_path.v - lane side path for image_pcie_capture
// ============================================================
// Pixel-domain tap -> lane_top -> result CDC. The RGB565 path
// is only observed; it is not modified.
// ============================================================
`timescale 1ns/1ps

module lane_capture_path #(
    parameter integer WIDTH       = 1280,
    parameter integer X_WIDTH     = 11,
    parameter integer Y_WIDTH     = 11,
    parameter integer SCAN_ROWS   = 21,
    parameter integer SCAN_Y0     = 528,
    parameter integer SCAN_STEP   = 7
) (
    input  wire                 pixel_clk,
    input  wire                 pixel_rst_n,

    input  wire                 vsync_i,
    input  wire                 de_i,
    input  wire [15:0]          rgb565_i,

    input  wire [10:0]          threshold_i,
    input  wire [10:0]          expected_left_i,
    input  wire [10:0]          expected_right_i,
    input  wire [10:0]          window_i,

    input  wire                 dst_clk,
    input  wire                 dst_rst_n,

    output wire                 dst_commit_o,
    output wire [31:0]          dst_frame_id_o,
    output wire [1:0]           dst_left_state_o,
    output wire [1:0]           dst_right_state_o,
    output wire signed [31:0]   dst_left_a_q8_o,
    output wire signed [31:0]   dst_right_a_q8_o,
    output wire signed [31:0]   dst_left_b_q8_o,
    output wire signed [31:0]   dst_right_b_q8_o,
    output wire                 dst_left_fit_valid_o,
    output wire                 dst_right_fit_valid_o,
    output wire [5:0]           debug_left_point_count_o,
    output wire [5:0]           debug_right_point_count_o,
    output wire                 debug_left_fit_valid_raw_o,
    output wire                 debug_right_fit_valid_raw_o,
    output wire [11:0]          debug_left_max_error_o,
    output wire [11:0]          debug_right_max_error_o,
    output wire [31:0]          debug_stream_word_o,
    output wire                 dst_busy_o
);

    wire frame_start;
    wire valid;
    wire [15:0] rgb565;
    wire eol;
    wire frame_done;
    wire [31:0] frame_id;

    wire [1:0] left_state;
    wire [1:0] right_state;
    wire signed [31:0] left_a_q8;
    wire signed [31:0] right_a_q8;
    wire signed [31:0] left_b_q8;
    wire signed [31:0] right_b_q8;
    wire left_fit_valid;
    wire right_fit_valid;
    wire result_valid;
    wire busy;

    reg          vsync_pixel;
    reg          de_pixel;
    reg [15:0]   rgb565_pixel;

    reg [4:0] coeff_row;
    reg       coeff_init_done;
    reg [31:0] coeff_start;
    reg [31:0] coeff_step;

    reg busy_meta;
    reg busy_sync;
    reg [15:0] debug_frame_done_count;
    wire [15:0] dbg_edge_pulses;
    wire [15:0] dbg_run_events;

    wire coeff_wr = !coeff_init_done;

    // Lane bring-up probe (2026-10-10): per-frame edge/run activity counters
    // exported through the right-b coefficient slot of the ABI window.
    assign debug_stream_word_o = {dbg_edge_pulses, dbg_run_events};

    // Capture the DVP stream on the falling edge so the rising-edge
    // lane core sees stable data despite the dedicated clock buffer skew.
    always @(negedge pixel_clk or negedge pixel_rst_n) begin
        if (!pixel_rst_n) begin
            vsync_pixel  <= 1'b0;
            de_pixel     <= 1'b0;
            rgb565_pixel <= 16'd0;
        end else begin
            vsync_pixel  <= vsync_i;
            de_pixel     <= de_i;
            rgb565_pixel <= rgb565_i;
        end
    end

    lane_stream_adapter #(
        .H_ACTIVE(WIDTH),
        .V_ACTIVE(720),
        .X_WIDTH(X_WIDTH),
        .Y_WIDTH(Y_WIDTH)
    ) u_stream_adapter (
        .clk(pixel_clk),
        .rst_n(pixel_rst_n),
        .vsync_i(vsync_pixel),
        .de_i(de_pixel),
        .rgb565_i(rgb565_pixel),
        .frame_start_o(frame_start),
        .valid_o(valid),
        .rgb565_o(rgb565),
        .eol_o(eol),
        .frame_done_o(frame_done),
        .frame_id_o(frame_id)
    );

    lane_top #(
        .WIDTH(WIDTH),
        .X_WIDTH(X_WIDTH),
        .Y_WIDTH(Y_WIDTH),
        .SCAN_ROWS(SCAN_ROWS),
        .SCAN_Y0(SCAN_Y0),
        .SCAN_STEP(SCAN_STEP)
    ) u_lane_top (
        .clk(pixel_clk),
        .rst_n(pixel_rst_n),
        .frame_start_i(frame_start),
        .valid_i(valid),
        .eol_i(eol),
        .rgb565_i(rgb565),
        .frame_done_i(frame_done),
        .threshold_i(threshold_i),
        .coeff_wr_i(coeff_wr),
        .coeff_row_i(coeff_row),
        .coeff_start_q16_i(coeff_start),
        .coeff_step_q16_i(coeff_step),
        .expected_left_i(expected_left_i),
        .expected_right_i(expected_right_i),
        .window_i(window_i),
        .left_state_o(left_state),
        .right_state_o(right_state),
        .left_a_q8_o(left_a_q8),
        .right_a_q8_o(right_a_q8),
        .left_b_q8_o(left_b_q8),
        .right_b_q8_o(right_b_q8),
        .left_fit_valid_o(left_fit_valid),
        .right_fit_valid_o(right_fit_valid),
        .debug_left_point_count_o(debug_left_point_count_o),
        .debug_right_point_count_o(debug_right_point_count_o),
        .debug_left_fit_valid_raw_o(debug_left_fit_valid_raw_o),
        .debug_right_fit_valid_raw_o(debug_right_fit_valid_raw_o),
        .debug_left_max_error_o(debug_left_max_error_o),
        .debug_right_max_error_o(debug_right_max_error_o),
        .dbg_edge_pulses_o(dbg_edge_pulses),
        .dbg_run_events_o(dbg_run_events),
        .result_valid_o(result_valid),
        .busy_o(busy)
    );

    lane_result_cdc u_result_cdc (
        .src_clk(pixel_clk),
        .src_rst_n(pixel_rst_n),
        .src_commit_i(result_valid),
        .src_frame_id_i(frame_id),
        .src_left_state_i(left_state),
        .src_right_state_i(right_state),
        .src_left_a_q8_i(left_a_q8),
        .src_right_a_q8_i(right_a_q8),
        .src_left_b_q8_i(left_b_q8),
        .src_right_b_q8_i(right_b_q8),
        .src_left_fit_valid_i(left_fit_valid),
        .src_right_fit_valid_i(right_fit_valid),
        .dst_clk(dst_clk),
        .dst_rst_n(dst_rst_n),
        .dst_commit_o(dst_commit_o),
        .dst_frame_id_o(dst_frame_id_o),
        .dst_left_state_o(dst_left_state_o),
        .dst_right_state_o(dst_right_state_o),
        .dst_left_a_q8_o(dst_left_a_q8_o),
        .dst_right_a_q8_o(dst_right_a_q8_o),
        .dst_left_b_q8_o(dst_left_b_q8_o),
        .dst_right_b_q8_o(dst_right_b_q8_o),
        .dst_left_fit_valid_o(dst_left_fit_valid_o),
        .dst_right_fit_valid_o(dst_right_fit_valid_o)
    );

    always @(posedge pixel_clk or negedge pixel_rst_n) begin
        if (!pixel_rst_n) begin
            debug_frame_done_count <= 16'd0;
        end else if (frame_done && debug_frame_done_count != 16'hffff) begin
            debug_frame_done_count <= debug_frame_done_count + 16'd1;
        end
    end

    always @(posedge pixel_clk or negedge pixel_rst_n) begin
        if (!pixel_rst_n) begin
            coeff_row       <= 5'd0;
            coeff_init_done <= 1'b0;
        end else if (!coeff_init_done) begin
            if (coeff_row == SCAN_ROWS - 1)
                coeff_init_done <= 1'b1;
            else
                coeff_row <= coeff_row + 1'b1;
        end
    end

    // 2026-10-11 empirical recalibration for the current camera framing,
    // scan band moved to y=528..668 (step 7) where the lane pair is wide
    // enough for the run filter (the old band started almost at the
    // vanishing point: 46 px gap -> x12 remap stretch -> runs over
    // MAX_RUN and every row dropped). Divider positions per row measured
    // on hardware; start = xL - 342*step, step = (xR-xL)/553.
    always @* begin
        coeff_start = 32'h01bb145b;
        coeff_step  = 32'h000041df;
        case (coeff_row)
            5'd0:  begin coeff_start = 32'h01bb145b; coeff_step = 32'h000041df; end
            5'd1:  begin coeff_start = 32'h01b65600; coeff_step = 32'h000046fc; end
            5'd2:  begin coeff_start = 32'h01b14efb; coeff_step = 32'h00004c68; end
            5'd3:  begin coeff_start = 32'h01abff4d; coeff_step = 32'h00005221; end
            5'd4:  begin coeff_start = 32'h01a666f6; coeff_step = 32'h00005829; end
            5'd5:  begin coeff_start = 32'h01a085f6; coeff_step = 32'h00005e80; end
            5'd6:  begin coeff_start = 32'h019a5c4c; coeff_step = 32'h00006524; end
            5'd7:  begin coeff_start = 32'h0193e9f9; coeff_step = 32'h00006c17; end
            5'd8:  begin coeff_start = 32'h018d2efc; coeff_step = 32'h00007359; end
            5'd9:  begin coeff_start = 32'h01862b57; coeff_step = 32'h00007ae8; end
            5'd10: begin coeff_start = 32'h017edf08; coeff_step = 32'h000082c6; end
            5'd11: begin coeff_start = 32'h01774a10; coeff_step = 32'h00008af3; end
            5'd12: begin coeff_start = 32'h016f6c6f; coeff_step = 32'h0000936d; end
            5'd13: begin coeff_start = 32'h01674624; coeff_step = 32'h00009c36; end
            5'd14: begin coeff_start = 32'h015ed730; coeff_step = 32'h0000a54e; end
            5'd15: begin coeff_start = 32'h01561f93; coeff_step = 32'h0000aeb3; end
            5'd16: begin coeff_start = 32'h014d1f4c; coeff_step = 32'h0000b867; end
            5'd17: begin coeff_start = 32'h0143d65d; coeff_step = 32'h0000c269; end
            5'd18: begin coeff_start = 32'h013a44c4; coeff_step = 32'h0000ccba; end
            5'd19: begin coeff_start = 32'h01306a81; coeff_step = 32'h0000d759; end
            5'd20: begin coeff_start = 32'h01264796; coeff_step = 32'h0000e246; end
            default: begin coeff_start = 32'h01bb145b; coeff_step = 32'h000041df; end
        endcase
    end

    always @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n) begin
            busy_meta <= 1'b0;
            busy_sync <= 1'b0;
        end else begin
            busy_meta <= busy;
            busy_sync <= busy_meta;
        end
    end

    assign dst_busy_o = busy_sync;

endmodule
