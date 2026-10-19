`default_nettype none

/*
 * Copyright (c) 2026 Chris Wu and Daniel Wang
 * SPDX-License-Identifier: Apache-2.0
 */

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

    assign uio_out = {6'd0, is_valid_output_wire, 1'b0};

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
    output wire [13:0] D_out,
    output wire [13:0] D_load,
    output wire [13:0] Read_A,
    output wire [13:0] Read_B,
    output wire [13:0] Write_A_prime,
    output wire [13:0] Write_B_prime
);

    localparam [1:0] S_IDLE_LOAD = 2'b00,
                     S_COMPUTE   = 2'b01,
                     S_OUTPUT    = 2'b10;

    reg [2:0] load_count;
    reg [3:0] compute_count;
    reg [3:0] output_count;
    reg [1:0] twiddle_sel;

    // Load real sample from ui_in[6:0], imaginary part is 0
    assign D_load = {ui_in[6:0], 7'sd0};

    // Bit-reversed address mapping for DIT input loading: 0, 4, 2, 6, 1, 5, 3, 7
    function [2:0] bit_rev;
        input [2:0] idx;
        begin
            bit_rev = {idx[0], idx[1], idx[2]};
        end
    endfunction

    // FSM and Datapath Scheduling combinational logic
    always @(*) begin
        Addr_A     = 3'd0;
        Addr_B     = 3'd0;
        Shift_Ctrl = 2'd1; // Shift right by 1 per stage to prevent overflow
        twiddle_sel = 2'd0;
        WE_load = 1'b0;
        WE_math = 1'b0;

        case (Phase_State)
            S_IDLE_LOAD: begin
                Addr_A = bit_rev(load_count);
                WE_load = rst_n && uio_in[0];
            end
            S_COMPUTE: begin
                WE_math = rst_n;
                case (compute_count[3:2])
                    2'd0: begin // (0,1), (2,3), (4,5), (6,7): W0
                        Addr_A = {compute_count[1:0], 1'b0};
                        Addr_B = {compute_count[1:0], 1'b1};
                    end
                    2'd1: begin // (0,2), (1,3), (4,6), (5,7): W0,W2,W0,W2
                        Addr_A = {compute_count[1], 1'b0, compute_count[0]};
                        Addr_B = {compute_count[1], 1'b1, compute_count[0]};
                        twiddle_sel = {compute_count[0], 1'b0};
                    end
                    2'd2: begin // (0,4), (1,5), (2,6), (3,7): W0,W1,W2,W3
                        Addr_A = {1'b0, compute_count[1:0]};
                        Addr_B = {1'b1, compute_count[1:0]};
                        twiddle_sel = compute_count[1:0];
                    end
                    default: begin
                        WE_math = 1'b0;
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

    // Combinational results are captured by reg_file at the same edge that
    // advances compute_count; no delayed results or write addresses are needed.
    fft_butterfly butterfly_inst (
        .Read_A(Read_A),
        .Read_B(Read_B),
        .Shift_Ctrl(Shift_Ctrl),
        .Twiddle_Index(twiddle_sel),
        .Write_A_prime(Write_A_prime),
        .Write_B_prime(Write_B_prime)
    );

    // FSM Sequential Logic
    always @(posedge clk) begin
        if (!rst_n) begin
            Phase_State     <= S_IDLE_LOAD;
            load_count      <= 3'd0;
            compute_count   <= 4'd0;
            output_count    <= 4'd0;
            is_valid_output <= 1'b0;
            uo_out          <= 8'd0;
        end else begin
            is_valid_output <= 1'b0;
            uo_out          <= 8'd0;

            case (Phase_State)
                S_IDLE_LOAD: begin
                    if (uio_in[0]) begin
                        if (load_count == 3'd7) begin
                            load_count    <= 3'd0;
                            Phase_State   <= S_COMPUTE;
                            compute_count <= 4'd0;
                        end else begin
                            load_count <= load_count + 3'd1;
                        end
                    end
                end

                S_COMPUTE: begin
                    if (compute_count == 4'd11) begin
                        Phase_State  <= S_OUTPUT;
                        output_count <= 4'd0;
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
                    output_count <= output_count + 4'd1;
                end

                default: begin
                    Phase_State <= S_IDLE_LOAD;
                    load_count <= 3'd0;
                    compute_count <= 4'd0;
                    output_count <= 4'd0;
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

    // 1/sqrt(2) ~= 0.10110101 binary = 181/256 (eight fractional bits).
    // Round to nearest, ties toward +infinity. Pre-adds are in [-128, 128],
    // so the product magnitude is at most 23168 and fits in 16 signed bits.
    function signed [8:0] times_sqrt_half;
        input signed [8:0] value;
        reg signed [15:0] extended_value;
        reg signed [15:0] product;
        reg signed [15:0] rounded_product;
        begin
            extended_value = {{7{value[8]}}, value};
            product = (extended_value <<< 7) + (extended_value <<< 5)
                    + (extended_value <<< 4)
                    + (extended_value <<< 2) + extended_value;
            rounded_product = (product + 16'sd128) >>> 8;
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