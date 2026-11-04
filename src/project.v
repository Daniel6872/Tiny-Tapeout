`default_nettype none

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
                mem[i] <= 14'd0;
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
        b_real = {Read_B[13], Read_B[13:7]};
        b_imag = {Read_B[6], Read_B[6:0]};
        sum_b = b_real + b_imag;
        diff_b = b_imag - b_real;
        scaled_sum_b = times_sqrt_half(sum_b);
        scaled_diff_b = times_sqrt_half(diff_b);
        twiddle_real = b_real;
        twiddle_imag = b_imag;
        Write_A_prime = 14'd0;
        Write_B_prime = 14'd0;

        // Stage 0: W0; stage 1: W0/W2; stage 2: W0/W1/W2/W3.
        case (Twiddle_Index)
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

// Register file + butterfly datapath. Read_A/Read_B feed the butterfly, and its
// combinational results are written back to Addr_A/Addr_B on the WE_math edge.
module fft_datapath (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        WE_load,
    input  wire        WE_math,
    input  wire [2:0]  Addr_A,
    input  wire [2:0]  Addr_B,
    input  wire [13:0] D_load,
    input  wire [1:0]  Twiddle_Index,
    input  wire [1:0]  Shift_Ctrl,
    output wire [13:0] D_out
);

    wire [13:0] Read_A;
    wire [13:0] Read_B;
    wire [13:0] Write_A_prime;
    wire [13:0] Write_B_prime;

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

    fft_butterfly butterfly_inst (
        .Read_A(Read_A),
        .Read_B(Read_B),
        .Twiddle_Index(Twiddle_Index),
        .Shift_Ctrl(Shift_Ctrl),
        .Write_A_prime(Write_A_prime),
        .Write_B_prime(Write_B_prime)
    );

endmodule
`default_nettype wire
