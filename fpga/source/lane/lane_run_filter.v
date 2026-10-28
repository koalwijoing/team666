// ============================================================
// lane_run_filter.v - lane run filtering with wide-run exclusion
// ============================================================
// This module buffers one remapped scan row, finds wide runs first, and
// then emits only narrow lane candidates outside the wide-run guard bands.
// It keeps the scan_runs streaming interface so lane_core can use it as a
// drop-in replacement while scan_runs remains available for regression.
// ============================================================
`timescale 1ns/1ps

module lane_run_filter #(
    parameter integer WIDTH    = 1280,
    parameter integer X_WIDTH  = 11,
    parameter integer MIN_RUN  = 3,
    parameter integer MAX_RUN  = 28,
    parameter integer WIDE_MIN = 30,
    parameter integer GUARD    = 24,
    parameter integer MAX_WIDE = 16
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        row_start_i,
    input  wire [4:0]  row_idx_i,
    input  wire        bit_valid_i,
    input  wire        bit_i,
    input  wire        row_end_i,

    output reg         run_valid_o,
    output reg  [10:0] run_center_o,
    output reg  [7:0]  run_len_o,
    output reg  [4:0]  run_row_idx_o,
    output reg  [7:0]  run_count_o,
    output reg         row_done_o
);

    localparam [2:0] S_IDLE = 3'd0;
    localparam [2:0] S_LOAD = 3'd1;
    localparam [2:0] S_WIDE = 3'd2;
    localparam [2:0] S_EMIT = 3'd3;
    localparam [2:0] S_DONE = 3'd4;

    reg [2:0]        state;
    reg [X_WIDTH:0]   col_idx;
    reg [X_WIDTH:0]   scan_len;
    reg [4:0]         row_idx_lat;
    reg               in_run;
    reg [X_WIDTH-1:0] run_start;
    reg [7:0]         run_len;
    reg [4:0]         wide_count;
    reg               wide_overflow;
    reg [X_WIDTH-1:0] wide_center [0:MAX_WIDE-1];
    reg [7:0]         wide_width  [0:MAX_WIDE-1];
    reg               row_bits [0:WIDTH-1];

    integer i;

    function automatic [10:0] run_center;
        input [X_WIDTH-1:0] start;
        input [7:0] len;
        begin
            run_center = start + ((len - 1'b1) >> 1);
        end
    endfunction

    function automatic wide_overlaps;
        input [10:0] center;
        input [7:0]  len;
        integer k;
        reg [11:0] half;
        reg [11:0] distance;
        reg [11:0] guard_span;
        begin
            wide_overlaps = 1'b0;
            for (k = 0; k < MAX_WIDE; k = k + 1) begin
                if (k < wide_count) begin
                    half = wide_width[k] >> 1;
                    guard_span = half + GUARD;
                    if (center >= wide_center[k])
                        distance = center - wide_center[k];
                    else
                        distance = wide_center[k] - center;
                    if (distance <= guard_span + (len >> 1))
                        wide_overlaps = 1'b1;
                end
            end
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_IDLE;
            col_idx       <= {X_WIDTH{1'b0}};
            scan_len      <= {(X_WIDTH+1){1'b0}};
            row_idx_lat   <= 5'd0;
            in_run        <= 1'b0;
            run_start     <= {X_WIDTH{1'b0}};
            run_len       <= 8'd0;
            wide_count    <= 5'd0;
            wide_overflow <= 1'b0;
            run_valid_o   <= 1'b0;
            run_center_o  <= 11'd0;
            run_len_o     <= 8'd0;
            run_row_idx_o <= 5'd0;
            run_count_o   <= 8'd0;
            row_done_o    <= 1'b0;
            for (i = 0; i < WIDTH; i = i + 1)
                row_bits[i] <= 1'b0;
            for (i = 0; i < MAX_WIDE; i = i + 1) begin
                wide_center[i] <= 11'd0;
                wide_width[i]  <= 8'd0;
            end
        end else begin
            run_valid_o <= 1'b0;
            row_done_o  <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (row_start_i) begin
                        col_idx       <= {X_WIDTH{1'b0}};
                        scan_len      <= {(X_WIDTH+1){1'b0}};
                        row_idx_lat   <= row_idx_i;
                        in_run        <= 1'b0;
                        run_start     <= {X_WIDTH{1'b0}};
                        run_len       <= 8'd0;
                        wide_count    <= 5'd0;
                        wide_overflow <= 1'b0;
                        run_count_o   <= 8'd0;
                        state         <= S_LOAD;
                    end
                end

                S_LOAD: begin
                    if (bit_valid_i) begin
                        row_bits[col_idx] <= bit_i;
                        col_idx <= col_idx + 1'b1;
                        scan_len <= scan_len + 1'b1;
                    end

                    if (row_end_i) begin
                        col_idx <= {X_WIDTH{1'b0}};
                        in_run  <= 1'b0;
                        run_len <= 8'd0;
                        state   <= S_WIDE;
                    end
                end

                S_WIDE: begin
                    if (col_idx < scan_len) begin
                        if (row_bits[col_idx]) begin
                            if (!in_run) begin
                                in_run    <= 1'b1;
                                run_start <= col_idx;
                                run_len   <= 8'd1;
                            end else begin
                                run_len <= run_len + 1'b1;
                            end
                        end else if (in_run) begin
                            in_run <= 1'b0;
                            if (run_len >= WIDE_MIN) begin
                                if (wide_count < MAX_WIDE) begin
                                    wide_center[wide_count] <= run_center(run_start, run_len);
                                    wide_width[wide_count]  <= run_len;
                                    wide_count <= wide_count + 1'b1;
                                end else begin
                                    wide_overflow <= 1'b1;
                                end
                            end
                            run_len <= 8'd0;
                        end
                        col_idx <= col_idx + 1'b1;
                    end else begin
                        if (in_run && (run_len >= WIDE_MIN)) begin
                            if (wide_count < MAX_WIDE) begin
                                wide_center[wide_count] <= run_center(run_start, run_len);
                                wide_width[wide_count]  <= run_len;
                                wide_count <= wide_count + 1'b1;
                            end else begin
                                wide_overflow <= 1'b1;
                            end
                        end
                        col_idx <= {X_WIDTH{1'b0}};
                        in_run  <= 1'b0;
                        run_len <= 8'd0;
                        state   <= S_EMIT;
                    end
                end

                S_EMIT: begin
                    if (col_idx < scan_len) begin
                        if (row_bits[col_idx]) begin
                            if (!in_run) begin
                                in_run    <= 1'b1;
                                run_start <= col_idx;
                                run_len   <= 8'd1;
                            end else begin
                                run_len <= run_len + 1'b1;
                            end
                        end else if (in_run) begin
                            in_run <= 1'b0;
                            if ((run_len >= MIN_RUN) && (run_len <= MAX_RUN) &&
                                !wide_overflow &&
                                !wide_overlaps(run_center(run_start, run_len), run_len)) begin
                                run_valid_o   <= 1'b1;
                                run_center_o  <= run_center(run_start, run_len);
                                run_len_o     <= run_len;
                                run_row_idx_o <= row_idx_lat;
                                run_count_o   <= run_count_o + 1'b1;
                            end
                            run_len <= 8'd0;
                        end
                        col_idx <= col_idx + 1'b1;
                    end else begin
                        if (in_run &&
                            (run_len >= MIN_RUN) && (run_len <= MAX_RUN) &&
                            !wide_overflow &&
                            !wide_overlaps(run_center(run_start, run_len), run_len)) begin
                            run_valid_o   <= 1'b1;
                            run_center_o  <= run_center(run_start, run_len);
                            run_len_o     <= run_len;
                            run_row_idx_o <= row_idx_lat;
                            run_count_o   <= run_count_o + 1'b1;
                        end
                        in_run <= 1'b0;
                        run_len <= 8'd0;
                        state <= S_DONE;
                    end
                end

                S_DONE: begin
                    row_done_o <= 1'b1;
                    state      <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
