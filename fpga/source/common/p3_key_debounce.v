// ============================================================
// p3_key_debounce.v - synchronize and debounce an active-low key
// ============================================================
`timescale 1ns/1ps

module p3_key_debounce #(
    parameter integer DEBOUNCE_CYCLES = 2_500_000
) (
    input  wire key_n_async,
    input  wire clk,
    input  wire rst_n,
    output reg  pressed,
    output reg  press_pulse
);

    reg [1:0] sync;
    reg [21:0] count;
    wire pressed_raw = ~sync[1];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync        <= 2'b11;
            count       <= 22'd0;
            pressed     <= 1'b0;
            press_pulse <= 1'b0;
        end else begin
            sync        <= {sync[0], key_n_async};
            press_pulse <= 1'b0;

            if (pressed_raw != pressed) begin
                if (count >= DEBOUNCE_CYCLES - 1) begin
                    pressed     <= pressed_raw;
                    press_pulse <= pressed_raw;
                    count       <= 22'd0;
                end else begin
                    count <= count + 22'd1;
                end
            end else begin
                count <= 22'd0;
            end
        end
    end

endmodule
