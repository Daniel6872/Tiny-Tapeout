/*
 * Copyright (c) 2024 Your Name
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