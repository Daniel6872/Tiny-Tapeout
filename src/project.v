/*
 * Copyright (c) 2026 Chris Wu and Daniel Wang
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_example (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

  // Unused bidirectional outputs assigned to 0
  assign uio_out = 8'b0;
  assign uio_oe  = 8'b0;

  // List unused inputs to suppress compiler warnings (ena is not used)
  wire _unused = &{ena, 1'b0};

  // Instantiate your 8-bit programmable counter
  counter_8bit user_counter (
      .clk      (clk),
      .rst_n    (rst_n),
      .load     (ui_in[0]),    // Input pin 0 -> Load
      .en       (ui_in[1]),    // Input pin 1 -> Enable
      .oe       (ui_in[2]),    // Input pin 2 -> Output Enable
      .data_in  (uio_in),      // 8-bit input bus -> Parallel Load Data
      .data_out (uo_out)       // 8-bit output bus -> Counter Output
  );

endmodule

module fft_butterfly (
    input  wire [13:0] Read_A,
    input  wire [13:0] Read_B,
    input  wire [1:0]  Twiddle_Index,
    input  wire [1:0]  Shift_Ctrl,
    output reg  [13:0] Write_A_prime,
    output reg  [13:0] Write_B_prime
);
    reg signed [8:0] a_real, a_imag;
    reg signed [8:0] b_real, b_imag;
    reg signed [8:0] twiddle_real, twiddle_imag;
    reg signed [8:0] factor_real, factor_imag;

    // 45/64 approximates 1/sqrt(2). Round to nearest, ties toward +infinity.
    // W1/W3 pre-adds can reach +128, so keep nine signed bits.
    function signed [8:0] times_sqrt_half;
        input signed [8:0] value;
        reg signed [13:0] extended_value;
        reg signed [13:0] product;
        reg signed [13:0] rounded_product;
        begin
            extended_value = {{5{value[8]}}, value};
            product = (extended_value <<< 5) + (extended_value <<< 3)
                    + (extended_value <<< 2) + extended_value;
            rounded_product = (product + 14'sd32) >>> 6;
            times_sqrt_half = rounded_product[8:0];
        end
    endfunction

    // Shift_Ctrl selects division by 1, 2, 4, or 8. Use 1 for every FFT stage.
    // Round to nearest (ties toward +infinity), then saturate instead of wrapping.
    // Saturation also protects arbitrary complex inputs at the negative endpoint.
    function [6:0] scale_and_saturate;
        input signed [8:0] value;
        input [1:0] shift;
        reg signed [8:0] scaled;
        begin
            scaled = value;
            case (shift)
                2'd0: scaled = value;
                2'd1: scaled = (value + 9'sd1) >>> 1;
                2'd2: scaled = (value + 9'sd2) >>> 2;
                2'd3: scaled = (value + 9'sd4) >>> 3;
            endcase
            if (scaled > 9'sd63)
                scale_and_saturate = 7'b0111111;
            else if (scaled < -9'sd64)
                scale_and_saturate = 7'b1000000;
            else
                scale_and_saturate = scaled[6:0];
        end
    endfunction

    always @(*) begin
        a_real = {{2{Read_A[13]}}, Read_A[13:7]};
        a_imag = {{2{Read_A[6]}}, Read_A[6:0]};
        b_real = {{2{Read_B[13]}}, Read_B[13:7]};
        b_imag = {{2{Read_B[6]}}, Read_B[6:0]};
        twiddle_real = b_real;
        twiddle_imag = b_imag;
        factor_real = 9'sd0;
        factor_imag = 9'sd0;
        Write_A_prime = 14'd0;
        Write_B_prime = 14'd0;

        // Stage 0: W0; stage 1: W0/W2; stage 2: W0/W1/W2/W3.
        case (Twiddle_Index)
            2'd0: begin
                twiddle_real = b_real;
                twiddle_imag = b_imag;
            end
            2'd1: begin
                factor_real = b_real + b_imag;
                factor_imag = b_imag - b_real;
            end
            2'd2: begin
                twiddle_real = b_imag;
                twiddle_imag = -b_real;
            end
            2'd3: begin
                factor_real = b_imag - b_real;
                factor_imag = -b_real - b_imag;
            end
        endcase

        // Share the same two constant-coefficient networks for W1 and W3.
        if (Twiddle_Index[0]) begin
            twiddle_real = times_sqrt_half(factor_real);
            twiddle_imag = times_sqrt_half(factor_imag);
        end

        Write_A_prime = {
            scale_and_saturate(a_real + twiddle_real, Shift_Ctrl),
            scale_and_saturate(a_imag + twiddle_imag, Shift_Ctrl)
        };
        Write_B_prime = {
            scale_and_saturate(a_real - twiddle_real, Shift_Ctrl),
            scale_and_saturate(a_imag - twiddle_imag, Shift_Ctrl)
        };
    end
endmodule