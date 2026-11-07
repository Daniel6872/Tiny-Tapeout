`default_nettype none

module reg_file (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        we_load,
    input  wire        we_math,
    input  wire [2:0]  addr_a,
    input  wire [2:0]  addr_b,
    input  wire [13:0] d_load,
    input  wire [13:0] write_a_prime,
    input  wire [13:0] write_b_prime,
    output wire [13:0] read_a,
    output wire [13:0] read_b,
    output wire [13:0] d_out
);

    reg [13:0] mem [7:0];
    integer i;

    always @(posedge clk) begin
        if (!rst_n) begin
            for (i = 0; i < 8; i = i + 1) begin
                mem[i] <= 14'd0;
            end
        end else if (we_load) begin
            mem[addr_a] <= d_load;
        end else if (we_math) begin
            mem[addr_a] <= write_a_prime;
            mem[addr_b] <= write_b_prime;
        end
    end

    assign read_a = mem[addr_a];
    assign read_b = mem[addr_b];
    assign d_out  = mem[addr_a];

endmodule

module fft_butterfly (
    input  wire [13:0] read_a,
    input  wire [13:0] read_b,
    input  wire [1:0]  twiddle_index,
    input  wire [1:0]  shift_ctrl,
    output reg  [13:0] write_a_prime,
    output reg  [13:0] write_b_prime
);
    reg signed [8:0] a_real, a_imag;
    reg signed [7:0] b_real, b_imag;
    reg signed [7:0] twiddle_real, twiddle_imag;
    reg signed [7:0] sum_b, diff_b;
    reg signed [7:0] scaled_sum_b, scaled_diff_b;

    // 1/sqrt(2) ~= 0.10110101 binary = 181/256 (eight fractional bits).
    // Round to nearest, ties toward +infinity. Pre-adds are in [-128, 127],
    // so the product magnitude is at most 23168 and fits in 16 signed bits.
    function signed [7:0] times_sqrt_half;
        input signed [7:0] value;
        reg signed [9:0] triple;
        reg signed [13:0] extended_triple;
        reg signed [13:0] times_45;
        reg signed [15:0] extended_value;
        reg signed [15:0] product;
        reg signed [15:0] rounded_product;
        begin
            extended_value = {{8{value[7]}}, value};
            // 181*x = 4*(16*(3*x) - 3*x) + x: three add/subtract operations.
            triple = $signed({{2{value[7]}}, value}) + ($signed({{2{value[7]}}, value}) <<< 1);
            extended_triple = {{4{triple[9]}}, triple};
            times_45 = (extended_triple <<< 4) - extended_triple;
            product = ($signed({{2{times_45[13]}}, times_45}) <<< 2) + extended_value;
            rounded_product = (product + 16'sd128) >>> 8;
            times_sqrt_half = rounded_product[7:0];
        end
    endfunction

    // shift_ctrl selects division by 1, 2, 4, or 8. Use 1 for every FFT stage.
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
        a_real = {{2{read_a[13]}}, read_a[13:7]};
        a_imag = {{2{read_a[6]}}, read_a[6:0]};
        b_real = {read_b[13], read_b[13:7]};
        b_imag = {read_b[6], read_b[6:0]};
        sum_b = b_real + b_imag;
        diff_b = b_imag - b_real;
        scaled_sum_b = times_sqrt_half(sum_b);
        scaled_diff_b = times_sqrt_half(diff_b);
        twiddle_real = b_real;
        twiddle_imag = b_imag;
        write_a_prime = 14'd0;
        write_b_prime = 14'd0;

        // Stage 0: W0; stage 1: W0/W2; stage 2: W0/W1/W2/W3.
        case (twiddle_index)
            2'd0: begin
                twiddle_real = b_real;
                twiddle_imag = b_imag;
            end
            2'd1: begin
                twiddle_real = scaled_sum_b;
                twiddle_imag = scaled_diff_b;
            end
            2'd2: begin
                twiddle_real = b_imag;
                twiddle_imag = -b_real;
            end
            2'd3: begin
                twiddle_real = scaled_diff_b;
                // q(-x) = -q(x), except the reachable half-way case x=-128.
                // Its +infinity tie rounding gives q(128)=91, q(-128)=-90.
                twiddle_imag = -scaled_sum_b + ((sum_b == -8'sd128) ? 8'sd1 : 8'sd0);
            end
        endcase

        write_a_prime = {
            scale_and_saturate(a_real + twiddle_real, shift_ctrl),
            scale_and_saturate(a_imag + twiddle_imag, shift_ctrl)
        };
        write_b_prime = {
            scale_and_saturate(a_real - twiddle_real, shift_ctrl),
            scale_and_saturate(a_imag - twiddle_imag, shift_ctrl)
        };
    end
endmodule

// Register file + butterfly datapath. read_a/read_b feed the butterfly, and its
// combinational results are written back to addr_a/addr_b on the we_math edge.
module fft_datapath (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        we_load,
    input  wire        we_math,
    input  wire [2:0]  addr_a,
    input  wire [2:0]  addr_b,
    input  wire [13:0] d_load,
    input  wire [1:0]  twiddle_index,
    input  wire [1:0]  shift_ctrl,
    output wire [13:0] d_out
);

    wire [13:0] read_a;
    wire [13:0] read_b;
    wire [13:0] write_a_prime;
    wire [13:0] write_b_prime;

    reg_file reg_file_inst (
        .clk(clk),
        .rst_n(rst_n),
        .we_load(we_load),
        .we_math(we_math),
        .addr_a(addr_a),
        .addr_b(addr_b),
        .d_load(d_load),
        .write_a_prime(write_a_prime),
        .write_b_prime(write_b_prime),
        .read_a(read_a),
        .read_b(read_b),
        .d_out(d_out)
    );

    fft_butterfly butterfly_inst (
        .read_a(read_a),
        .read_b(read_b),
        .twiddle_index(twiddle_index),
        .shift_ctrl(shift_ctrl),
        .write_a_prime(write_a_prime),
        .write_b_prime(write_b_prime)
    );

endmodule

// Tiny Tapeout I/O side of the FFT datapath. The FSM owns the register-file
// addresses and write enables; it must hold each output-bin address for two
// cycles during unload (real first, then imaginary).
module fft_io_wrapper (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [7:0]  ui_in,
    input  wire [1:0]  Phase_State,
    input  wire [13:0] D_out,
    output wire [13:0] D_load,
    output reg  [7:0]  uo_out,
    output wire [7:0]  uio_out,
    output wire [7:0]  uio_oe
);
    localparam [1:0] PHASE_LOAD    = 2'd0;
    localparam [1:0] PHASE_COMPUTE = 2'd1;
    localparam [1:0] PHASE_UNLOAD  = 2'd2;

    reg [3:0] stream_cycle;
    wire is_valid_input;
    wire is_valid_output;

    assign D_load = {ui_in[6:0], 7'd0};
    assign is_valid_input = (Phase_State == PHASE_LOAD);
    assign is_valid_output = (Phase_State == PHASE_UNLOAD);

    // Expose phase-valid indicators on uio[0] and uio[1].
    assign uio_out = {6'b0, is_valid_output, is_valid_input};
    assign uio_oe = 8'b00000011;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            stream_cycle <= 4'd0;
        else if (Phase_State != PHASE_UNLOAD)
            stream_cycle <= 4'd0;
        else if (stream_cycle != 4'd15)
            stream_cycle <= stream_cycle + 1'b1;
    end

    always @(*) begin
        uo_out = 8'd0;
        if (Phase_State == PHASE_UNLOAD) begin
            if (stream_cycle[0])
                uo_out = {1'b1, D_out[6:0]};
            else
                uo_out = {1'b0, D_out[13:7]};
        end
    end

endmodule

// FSM controller. One 4-bit step counter sequences every phase:
//   LOAD    step 0..7 : each rising edge with is_valid_input high writes the
//                       sample to address bitrev(step), so x[0..7] land at
//                       0,4,2,6,1,5,3,7.
//   COMPUTE step 0..11: one butterfly per clock. step[3:2] is the stage and
//                       step[1:0] the butterfly within it:
//                         stage 0: (0,1,W0) (2,3,W0) (4,5,W0) (6,7,W0)
//                         stage 1: (0,2,W0) (1,3,W2) (4,6,W0) (5,7,W2)
//                         stage 2: (0,4,W0) (1,5,W1) (2,6,W2) (3,7,W3)
//                       as (addr_a, addr_b, twiddle). Results are written at the
//                       same edge that advances step, so each butterfly reads the
//                       previous one's results with no pipeline hazard.
//   OUTPUT  step 0..15: is_valid_output is high and d_out = X[addr_a], with
//                       addr_a = step[3:1] in natural order; output_imag =
//                       step[0] selects the real (0) or imaginary (1) part.
// phase_state is exported for fft_io_wrapper; its encoding matches PHASE_LOAD,
// PHASE_COMPUTE, and PHASE_UNLOAD there.
// Writes are decoded from the phase, so we_load and we_math are never both high,
// and addr_a != addr_b whenever we_math is high.
module fft_controller (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       is_valid_input,
    output reg  [1:0] phase_state,
    output reg        we_load,
    output reg        we_math,
    output reg  [2:0] addr_a,
    output reg  [2:0] addr_b,
    output reg  [1:0] twiddle_index,
    output wire [1:0] shift_ctrl,
    output wire       is_valid_output,
    output wire       output_imag
);

    localparam [1:0] phase_load    = 2'd0,
                     phase_compute = 2'd1,
                     phase_output  = 2'd2;

    reg [3:0] step;

    // Halve every stage so the 7-bit outputs are X[k]/8 and cannot overflow.
    assign shift_ctrl      = 2'd1;
    assign is_valid_output = (phase_state == phase_output);
    assign output_imag     = step[0];

    always @(*) begin
        we_load       = 1'b0;
        we_math       = 1'b0;
        addr_a        = 3'd0;
        addr_b        = 3'd0;
        twiddle_index = 2'd0;

        case (phase_state)
            phase_load: begin
                we_load = is_valid_input;
                addr_a  = {step[0], step[1], step[2]};
            end
            phase_compute: begin
                we_math = 1'b1;
                case (step[3:2])
                    2'd0: begin
                        addr_a = {step[1:0], 1'b0};
                        addr_b = {step[1:0], 1'b1};
                    end
                    2'd1: begin
                        addr_a        = {step[1], 1'b0, step[0]};
                        addr_b        = {step[1], 1'b1, step[0]};
                        twiddle_index = {step[0], 1'b0};
                    end
                    2'd2: begin
                        addr_a        = {1'b0, step[1:0]};
                        addr_b        = {1'b1, step[1:0]};
                        twiddle_index = step[1:0];
                    end
                    default: begin
                        we_math = 1'b0;
                    end
                endcase
            end
            phase_output: begin
                addr_a = step[3:1];
            end
            default: begin
            end
        endcase
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            phase_state <= phase_load;
            step        <= 4'd0;
        end else begin
            case (phase_state)
                phase_load: begin
                    if (is_valid_input) begin
                        if (step == 4'd7) begin
                            phase_state <= phase_compute;
                            step        <= 4'd0;
                        end else begin
                            step <= step + 4'd1;
                        end
                    end
                end
                phase_compute: begin
                    if (step == 4'd11) begin
                        phase_state <= phase_output;
                        step        <= 4'd0;
                    end else begin
                        step <= step + 4'd1;
                    end
                end
                phase_output: begin
                    // step wraps from 15 to 0, ready for the next frame's first sample.
                    step <= step + 4'd1;
                    if (step == 4'd15) begin
                        phase_state <= phase_load;
                    end
                end
                default: begin
                    phase_state <= phase_load;
                    step        <= 4'd0;
                end
            endcase
        end
    end

endmodule

// Tiny Tapeout top level: FSM controller, datapath, and fft_io_wrapper pins.
//   ui_in[6:0]  signed 7-bit real sample; ui_in[7] is ignored
//   uio_out[0]  high throughout LOAD: ui_in is sampled at every rising edge
//   uio_out[1]  high throughout OUTPUT: uo_out = {0, real} then {1, imag}
//   uo_out      16 bytes, X[0]..X[7] in natural order; 0 outside OUTPUT
//   uio_in      ignored
// Frames repeat every 36 cycles: 8 load, 12 compute, and 16 output.
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

    wire [1:0]  phase_state;
    wire        we_load;
    wire        we_math;
    wire [2:0]  addr_a;
    wire [2:0]  addr_b;
    wire [1:0]  twiddle_index;
    wire [1:0]  shift_ctrl;
    wire        is_valid_output;
    wire        output_imag;
    wire [13:0] d_load;
    wire [13:0] d_out;

    fft_controller controller_inst (
        .clk(clk),
        .rst_n(rst_n),
        // fft_io_wrapper raises uio_out[0] for all of LOAD and takes a sample
        // every cycle, so the FSM loads on each LOAD cycle.
        .is_valid_input(uio_out[0]),
        .phase_state(phase_state),
        .we_load(we_load),
        .we_math(we_math),
        .addr_a(addr_a),
        .addr_b(addr_b),
        .twiddle_index(twiddle_index),
        .shift_ctrl(shift_ctrl),
        .is_valid_output(is_valid_output),
        .output_imag(output_imag)
    );

    fft_datapath datapath_inst (
        .clk(clk),
        .rst_n(rst_n),
        .we_load(we_load),
        .we_math(we_math),
        .addr_a(addr_a),
        .addr_b(addr_b),
        .d_load(d_load),
        .twiddle_index(twiddle_index),
        .shift_ctrl(shift_ctrl),
        .d_out(d_out)
    );

    fft_io_wrapper io_wrapper_inst (
        .clk(clk),
        .rst_n(rst_n),
        .ui_in(ui_in),
        .Phase_State(phase_state),
        .D_out(d_out),
        .D_load(d_load),
        .uo_out(uo_out),
        .uio_out(uio_out),
        .uio_oe(uio_oe)
    );

    // fft_io_wrapper derives its own valid and real/imaginary selects.
    wire _unused = &{ena, uio_in, is_valid_output, output_imag, 1'b0};

endmodule
`default_nettype wire
