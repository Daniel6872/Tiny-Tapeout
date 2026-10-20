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

`default_nettype wire
