`timescale 1ns / 1ps

module counter_8bit_tb;

    // Testbench signals
    reg        clk;
    reg        rst_n;
    reg        load;
    reg        en;
    reg        oe;
    reg  [7:0] data_in;
    wire [7:0] data_out;

    // Instantiate Unit Under Test (UUT)
    counter_8bit uut (
        .clk      (clk),
        .rst_n    (rst_n),
        .load     (load),
        .en       (en),
        .oe       (oe),
        .data_in  (data_in),
        .data_out (data_out)
    );

    // 100 MHz clock generation (10ns period)
    always #5 clk = ~clk;

    initial begin
        // Setup GTKWave waveform dump file
        $dumpfile("dump.vcd");
        $dumpvars(0, counter_8bit_tb);

        // Print real-time state changes to the terminal
        $monitor("Time=%0tns | rst_n=%b load=%b en=%b oe=%b | data_in=%h | data_out=%h",
                 $time, rst_n, load, en, oe, data_in, data_out);

        // Initial inputs
        clk     = 0;
        rst_n   = 0; // Assert reset
        load    = 0;
        en      = 0;
        oe      = 1;
        data_in = 8'h00;

        // Release reset after 15ns
        #15 rst_n = 1;
        #10;

        // Test Counting
        en = 1;
        #40;

        // Test Synchronous Load
        load    = 1;
        data_in = 8'hA5;
        #10;
        load    = 0;
        #30;

        // Test Output Enable (Tri-State Output)
        oe = 0;
        #20;
        oe = 1;
        #20;

        // Test Asynchronous Reset
        rst_n = 0;
        #10;

        $display("--- Simulation Complete ---");
        $finish;
    end

endmodule