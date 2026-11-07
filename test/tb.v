`default_nettype none
`timescale 1ns / 1ps

/* This testbench just instantiates the module and makes some convenient wires
   that can be driven / tested by the cocotb test.py.
*/
module tb ();

  // Dump the signals to a FST file. You can view it with gtkwave or surfer.
  initial begin
    $dumpfile("tb.fst");
    $dumpvars(0, tb);
    #1;
  end

  // Wire up the inputs and outputs:
  reg clk;
  reg rst_n;
  reg ena;
  reg [7:0] ui_in;
  reg [7:0] uio_in;
  wire [7:0] uo_out;
  wire [7:0] uio_out;
  wire [7:0] uio_oe;
`ifdef GL_TEST
  wire VPWR = 1'b1;
  wire VGND = 1'b0;
`else
  reg unit_rf_rst_n;
  reg unit_rf_we_load;
  reg unit_rf_we_math;
  reg [2:0] unit_rf_addr_a;
  reg [2:0] unit_rf_addr_b;
  reg [13:0] unit_rf_d_load;
  reg [13:0] unit_rf_write_a;
  reg [13:0] unit_rf_write_b;
  wire [13:0] unit_rf_read_a;
  wire [13:0] unit_rf_read_b;
  wire [13:0] unit_rf_d_out;

  reg [13:0] unit_bf_read_a;
  reg [13:0] unit_bf_read_b;
  reg [1:0] unit_bf_twiddle;
  reg [1:0] unit_bf_shift;
  wire [13:0] unit_bf_write_a;
  wire [13:0] unit_bf_write_b;

  reg unit_dp_rst_n;
  reg unit_dp_we_load;
  reg unit_dp_we_math;
  reg [2:0] unit_dp_addr_a;
  reg [2:0] unit_dp_addr_b;
  reg [13:0] unit_dp_d_load;
  reg [1:0] unit_dp_twiddle;
  reg [1:0] unit_dp_shift;
  wire [13:0] unit_dp_d_out;

  reg unit_io_rst_n;
  reg [7:0] unit_io_ui_in;
  reg [1:0] unit_io_phase;
  reg [13:0] unit_io_d_out;
  wire [13:0] unit_io_d_load;
  wire [7:0] unit_io_uo_out;
  wire [7:0] unit_io_uio_out;
  wire [7:0] unit_io_uio_oe;

  reg unit_ctrl_rst_n;
  reg unit_ctrl_is_valid_input;
  wire [1:0] unit_ctrl_phase_state;
  wire unit_ctrl_we_load;
  wire unit_ctrl_we_math;
  wire [2:0] unit_ctrl_addr_a;
  wire [2:0] unit_ctrl_addr_b;
  wire [1:0] unit_ctrl_twiddle_index;
  wire [1:0] unit_ctrl_shift_ctrl;
  wire unit_ctrl_is_valid_output;
  wire unit_ctrl_output_imag;
`endif

  // Replace tt_um_example with your module name:
  tt_um_example user_project (

      // Include power ports for the Gate Level test:
`ifdef GL_TEST
      .VPWR(VPWR),
      .VGND(VGND),
`endif

      .ui_in  (ui_in),    // Dedicated inputs
      .uo_out (uo_out),   // Dedicated outputs
      .uio_in (uio_in),   // IOs: Input path
      .uio_out(uio_out),  // IOs: Output path
      .uio_oe (uio_oe),   // IOs: Enable path (active high: 0=input, 1=output)
      .ena    (ena),      // enable - goes high when design is selected
      .clk    (clk),      // clock
      .rst_n  (rst_n)     // not reset
  );

`ifndef GL_TEST
  reg_file unit_reg_file (
      .clk(clk),
      .rst_n(unit_rf_rst_n),
      .we_load(unit_rf_we_load),
      .we_math(unit_rf_we_math),
      .addr_a(unit_rf_addr_a),
      .addr_b(unit_rf_addr_b),
      .d_load(unit_rf_d_load),
      .write_a_prime(unit_rf_write_a),
      .write_b_prime(unit_rf_write_b),
      .read_a(unit_rf_read_a),
      .read_b(unit_rf_read_b),
      .d_out(unit_rf_d_out)
  );

  fft_butterfly unit_fft_butterfly (
      .read_a(unit_bf_read_a),
      .read_b(unit_bf_read_b),
      .twiddle_index(unit_bf_twiddle),
      .shift_ctrl(unit_bf_shift),
      .write_a_prime(unit_bf_write_a),
      .write_b_prime(unit_bf_write_b)
  );

  fft_datapath unit_fft_datapath (
      .clk(clk),
      .rst_n(unit_dp_rst_n),
      .we_load(unit_dp_we_load),
      .we_math(unit_dp_we_math),
      .addr_a(unit_dp_addr_a),
      .addr_b(unit_dp_addr_b),
      .d_load(unit_dp_d_load),
      .twiddle_index(unit_dp_twiddle),
      .shift_ctrl(unit_dp_shift),
      .d_out(unit_dp_d_out)
  );

  fft_io_wrapper unit_fft_io_wrapper (
      .clk(clk),
      .rst_n(unit_io_rst_n),
      .ui_in(unit_io_ui_in),
      .Phase_State(unit_io_phase),
      .D_out(unit_io_d_out),
      .D_load(unit_io_d_load),
      .uo_out(unit_io_uo_out),
      .uio_out(unit_io_uio_out),
      .uio_oe(unit_io_uio_oe)
  );

  fft_controller unit_fft_controller (
      .clk(clk),
      .rst_n(unit_ctrl_rst_n),
      .is_valid_input(unit_ctrl_is_valid_input),
      .phase_state(unit_ctrl_phase_state),
      .we_load(unit_ctrl_we_load),
      .we_math(unit_ctrl_we_math),
      .addr_a(unit_ctrl_addr_a),
      .addr_b(unit_ctrl_addr_b),
      .twiddle_index(unit_ctrl_twiddle_index),
      .shift_ctrl(unit_ctrl_shift_ctrl),
      .is_valid_output(unit_ctrl_is_valid_output),
      .output_imag(unit_ctrl_output_imag)
  );
`endif

endmodule
