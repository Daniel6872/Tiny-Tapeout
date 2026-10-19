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

    // Unused inputs tie-off
    wire _unused = &{ena, ui_in[7], uio_in[7:1], 1'b0};

    // Bidirectional configuration: uio[1] = is_valid_output, others 0
    assign uio_oe = 8'b0000_0010;

    // Internal wires matching the system diagram architecture[cite: 1]
    wire [1:0]  Phase_State;
    wire [2:0]  Addr_A;
    wire [2:0]  Addr_B;
    wire        WE_load;
    wire        WE_math;
    wire [1:0]  Shift_Ctrl;
    wire [13:0] D_out;
    wire [13:0] D_load;
    wire [13:0] Write_A_prime;
    wire [13:0] Write_B_prime;
    wire [13:0] Read_A;
    wire [13:0] Read_B;
    wire        is_valid_output_wire;

    assign uio_out = {7'd0, is_valid_output_wire};

    // Instantiate Top Wrapper & FSM Controller[cite: 1]
    fft_top_wrapper top_wrapper_inst (
        .clk(clk),
        .rst_n(rst_n),
        .ui_in(ui_in),
        .uio_in(uio_in),
        .uo_out(uo_out),
        .is_valid_output(is_valid_output_wire),
        .Phase_State(Phase_State),
        .Addr_A(Addr_A),
        .Addr_B(Addr_B),
        .WE_load(WE_load),
        .WE_math(WE_math),
        .Shift_Ctrl(Shift_Ctrl),
        .D_out(D_out),
        .D_load(D_load),
        .Read_A(Read_A),
        .Read_B(Read_B),
        .Write_A_prime(Write_A_prime),
        .Write_B_prime(Write_B_prime)
    );

endmodule

// -----------------------------------------------------------------------------
// FSM Controller & Top Wrapper Logic
// -----------------------------------------------------------------------------
module fft_top_wrapper (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [7:0]  ui_in,
    input  wire [7:0]  uio_in,
    output reg  [7:0]  uo_out,
    output reg         is_valid_output,
    output reg  [1:0]  Phase_State,
    output reg  [2:0]  Addr_A,
    output reg  [2:0]  Addr_B,
    output reg         WE_load,
    output reg         WE_math,
    output reg  [1:0]  Shift_Ctrl,
    input  wire [13:0] D_out,
    output wire [13:0] D_load,
    input  wire [13:0] Read_A,
    input  wire [13:0] Read_B,
    output wire [13:0] Write_A_prime,
    output wire [13:0] Write_B_prime
);

    localparam [1:0] S_IDLE_LOAD = 2'b00,
                     S_COMPUTE   = 2'b01,
                     S_OUTPUT    = 2'b10;

    reg [3:0] load_count;
    reg [3:0] compute_count;
    reg [4:0] output_count;
    reg [2:0] load_addr_reg;
    reg [1:0] twiddle_sel;

    // Load real sample from ui_in[6:0], imaginary part is 0
    assign D_load = {ui_in[6:0], 7'sd0};

    // Bit-reversed address mapping for DIT input loading: 0, 4, 2, 6, 1, 5, 3, 7
    function [2:0] bit_rev;
        input [2:0] idx;
        begin
            case (idx)
                3'd0: bit_rev = 3'd0;
                3'd1: bit_rev = 3'd4;
                3'd2: bit_rev = 3'd2;
                3'd3: bit_rev = 3'd6;
                3'd4: bit_rev = 3'd1;
                3'd5: bit_rev = 3'd5;
                3'd6: bit_rev = 3'd3;
                3'd7: bit_rev = 3'd7;
                default: bit_rev = 3'd0;
            endcase
        end
    endfunction

    // FSM and Datapath Scheduling combinational logic
    always @(*) begin
        Addr_A     = 3'd0;
        Addr_B     = 3'd0;
        Shift_Ctrl = 2'd1; // Shift right by 1 per stage to prevent overflow

        case (Phase_State)
            S_IDLE_LOAD: begin
                Addr_A = load_addr_reg;
            end
            S_COMPUTE: begin
                case (compute_count[3:2])
                    2'd0: begin // Stage 0: W0 only
                        Shift_Ctrl  = 2'd1;
                        twiddle_sel = 2'b00;
                        case (compute_count[1:0])
                            2'd0: begin Addr_A = 3'd0; Addr_B = 3'd4; end
                            2'd1: begin Addr_A = 3'd1; Addr_B = 3'd5; end
                            2'd2: begin Addr_A = 3'd2; Addr_B = 3'd6; end
                            2'd3: begin Addr_A = 3'd3; Addr_B = 3'd7; end
                        endcase
                    end
                    2'd1: begin // Stage 1: W0, W2
                        Shift_Ctrl = 2'd1;
                        case (compute_count[1:0])
                            2'd0: begin Addr_A = 3'd0; Addr_B = 3'd2; twiddle_sel = 2'b00; end
                            2'd1: begin Addr_A = 3'd1; Addr_B = 3'd3; twiddle_sel = 2'b10; end
                            2'd2: begin Addr_A = 3'd4; Addr_B = 3'd6; twiddle_sel = 2'b00; end
                            2'd3: begin Addr_A = 3'd5; Addr_B = 3'd7; twiddle_sel = 2'b10; end
                        endcase
                    end
                    2'd2: begin // Stage 2: W0, W1, W2, W3
                        Shift_Ctrl = 2'd1;
                        case (compute_count[1:0])
                            2'd0: begin Addr_A = 3'd0; Addr_B = 3'd1; twiddle_sel = 2'b00; end
                            2'd1: begin Addr_A = 3'd2; Addr_B = 3'd3; twiddle_sel = 2'b01; end
                            2'd2: begin Addr_A = 3'd4; Addr_B = 3'd5; twiddle_sel = 2'b10; end
                            2'd3: begin Addr_A = 3'd6; Addr_B = 3'd7; twiddle_sel = 2'b11; end
                        endcase
                    end
                    default: begin
                        Addr_A = 3'd0;
                        Addr_B = 3'd1;
                    end
                endcase
            end
            S_OUTPUT: begin
                Addr_A = output_count[3:1];
            end
            default: begin
                Addr_A = 3'd0;
                Addr_B = 3'd0;
            end
        endcase
    end

    // Instantiate Register File (8 x 14 bit)[cite: 1]
    reg_file reg_file_inst (
        .clk(clk),
        .rst_n(rst_n),
        .WE_load(WE_load),
        .WE_math(WE_math),
        .Addr_A(Addr_A),
        .Addr_B(Addr_B),
        .D_load(D_load),
        .Write_A_prime(Write_A_prime),
        .Write_B_prime(Write_B_prime),
        .Read_A(Read_A),
        .Read_B(Read_B),
        .D_out(D_out)
    );

    // Instantiate Butterfly Unit[cite: 1]
    butterfly butterfly_inst (
        .clk(clk),
        .rst_n(rst_n),
        .Read_A(Read_A),
        .Read_B(Read_B),
        .Shift_Ctrl(Shift_Ctrl),
        .twiddle_sel(twiddle_sel),
        .Write_A_prime(Write_A_prime),
        .Write_B_prime(Write_B_prime)
    );

    // FSM Sequential Logic
    always @(posedge clk) begin
        if (!rst_n) begin
            Phase_State     <= S_IDLE_LOAD;
            load_count      <= 4'd0;
            load_addr_reg   <= 3'd0;
            compute_count   <= 4'd0;
            output_count    <= 5'd0;
            WE_load         <= 1'b0;
            WE_math         <= 1'b0;
            is_valid_output <= 1'b0;
            uo_out          <= 8'd0;
        end else begin
            WE_load         <= 1'b0;
            WE_math         <= 1'b0;
            is_valid_output <= 1'b0;

            case (Phase_State)
                S_IDLE_LOAD: begin
                    if (uio_in[0]) begin
                        WE_load       <= 1'b1;
                        load_addr_reg <= bit_rev(load_count[2:0]);
                        if (load_count == 4'd7) begin
                            load_count    <= 4'd0;
                            Phase_State   <= S_COMPUTE;
                            compute_count <= 4'd0;
                        end else begin
                            load_count <= load_count + 4'd1;
                        end
                    end
                end

                S_COMPUTE: begin
                    WE_math <= 1'b1;
                    if (compute_count == 4'd11) begin
                        Phase_State  <= S_OUTPUT;
                        output_count <= 5'd0;
                    end else begin
                        compute_count <= compute_count + 4'd1;
                    end
                end

                S_OUTPUT: begin
                    is_valid_output <= 1'b1;
                    if (output_count[0] == 1'b0) begin
                        // Real part sign-extended to 8 bits
                        uo_out <= { {1{D_out[13]}}, D_out[13:7] };
                    end else begin
                        // Imaginary part sign-extended to 8 bits
                        uo_out <= { {1{D_out[6]}}, D_out[6:0] };
                        if (Addr_A == 3'd7) begin
                            Phase_State <= S_IDLE_LOAD;
                        end
                    end
                    output_count <= output_count + 5'd1;
                end

                default: begin
                    Phase_State <= S_IDLE_LOAD;
                end
            endcase
        end
    end

endmodule

// -----------------------------------------------------------------------------
// Register File Module (8 x 14-bit)[cite: 1]
// -----------------------------------------------------------------------------
module reg_file (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        WE_load,
    input  wire        WE_math,
    input  wire [2:0]  Addr_A,
    input  wire [2:0]  Addr_B,
    input  wire [13:0] D_load,
    input  wire [13:0] Write_A_prime,
    input  wire [13:0] Write_B_prime,
    output wire [13:0] Read_A,
    output wire [13:0] Read_B,
    output wire [13:0] D_out
);

    reg [13:0] mem [7:0];
    integer i;

    always @(posedge clk) begin
        if (!rst_n) begin
            for (i = 0; i < 8; i = i + 1) begin
                mem[i] <= 14'sd0;
            end
        end else if (WE_load) begin
            mem[Addr_A] <= D_load;
        end else if (WE_math) begin
            mem[Addr_A] <= Write_A_prime;
            mem[Addr_B] <= Write_B_prime;
        end
    end

    assign Read_A = mem[Addr_A];
    assign Read_B = mem[Addr_B];
    assign D_out  = mem[Addr_A];

endmodule

// -----------------------------------------------------------------------------
// Butterfly Unit Module[cite: 1]
// -----------------------------------------------------------------------------
module butterfly (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [13:0] Read_A,
    input  wire [13:0] Read_B,
    input  wire [1:0]  Shift_Ctrl,
    input  wire [1:0]  twiddle_sel,
    output reg  [13:0] Write_A_prime,
    output reg  [13:0] Write_B_prime
);

    wire signed [6:0] a_re = Read_A[13:7];
    wire signed [6:0] a_im = Read_A[6:0];
    wire signed [6:0] b_re = Read_B[13:7];
    wire signed [6:0] b_im = Read_B[6:0];

    reg  signed [7:0] w_re_mult;
    reg  signed [7:0] w_im_mult;

    // 0.7071 approximation via shift-and-add: 45 / 64 (~0.703125)
    function signed [7:0] scale_0707;
        input signed [7:0] val;
        begin
            scale_0707 = (val * 8'sd45) >>> 6;
        end
    endfunction

    always @(*) begin
        case (twiddle_sel)
            2'b00: begin // W0 = 1
                w_re_mult = {b_re[6], b_re};
                w_im_mult = {b_im[6], b_im};
            end
            2'b01: begin // W1 = 0.7071 - j0.7071
                w_re_mult = scale_0707({b_re[6], b_re} + {b_im[6], b_im});
                w_im_mult = scale_0707({b_im[6], b_im} - {b_re[6], b_re});
            end
            2'b10: begin // W2 = -j (swaps and negations only)
                w_re_mult = {b_im[6], b_im};
                w_im_mult = -{b_re[6], b_re};
            end
            2'b11: begin // W3 = -0.7071 - j0.7071
                w_re_mult = scale_0707({b_im[6], b_im} - {b_re[6], b_re});
                w_im_mult = -scale_0707({b_re[6], b_re} + {b_im[6], b_im});
            end
            default: begin
                w_re_mult = {b_re[6], b_re};
                w_im_mult = {b_im[6], b_im};
            end
        endcase
    end

    wire signed [8:0] sum_re  = {a_re[6], a_re} + {w_re_mult[7], w_re_mult};
    wire signed [8:0] sum_im  = {a_im[6], a_im} + {w_im_mult[7], w_im_mult};
    wire signed [8:0] diff_re = {a_re[6], a_re} - {w_re_mult[7], w_re_mult};
    wire signed [8:0] diff_im = {a_im[6], a_im} - {w_im_mult[7], w_im_mult};

    wire signed [8:0] scaled_sum_re  = sum_re >>> Shift_Ctrl;
    wire signed [8:0] scaled_sum_im  = sum_im >>> Shift_Ctrl;
    wire signed [8:0] scaled_diff_re = diff_re >>> Shift_Ctrl;
    wire signed [8:0] scaled_diff_im = diff_im >>> Shift_Ctrl;

    function [6:0] saturate;
        input signed [8:0] val;
        begin
            if (val > 9'sd63)
                saturate = 7'sd63;
            else if (val < -9'sd64)
                saturate = -7'sd64;
            else
                saturate = val[6:0];
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            Write_A_prime <= 14'sd0;
            Write_B_prime <= 14'sd0;
        end else begin
            Write_A_prime <= {saturate(scaled_sum_re), saturate(scaled_sum_im)};
            Write_B_prime <= {saturate(scaled_diff_re), saturate(scaled_diff_im)};
        end
    end

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