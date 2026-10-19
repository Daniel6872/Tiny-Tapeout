`default_nettype none
`timescale 1ns / 1ps

module tb_butterfly;
    reg [13:0] Read_A;
    reg [13:0] Read_B;
    reg [1:0] Twiddle_Index;
    reg [1:0] Shift_Ctrl;
    wire [13:0] Write_A_prime;
    wire [13:0] Write_B_prime;

    fft_butterfly dut (
        .Read_A(Read_A),
        .Read_B(Read_B),
        .Twiddle_Index(Twiddle_Index),
        .Shift_Ctrl(Shift_Ctrl),
        .Write_A_prime(Write_A_prime),
        .Write_B_prime(Write_B_prime)
    );
endmodule
