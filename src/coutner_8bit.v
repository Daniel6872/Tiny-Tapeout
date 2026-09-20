module counter_8bit(

    input wire clk,
    input wire rst_n, //Async reset (active low: 0 = reset, 1 = run)
    input wire load, //Synchronous load signal
    input wire en, //Counter load signal
    input wire oe,//Ouput enable signal
    input wire[7:0] data_in, //8 bit input
    output wire[7:0] data_out //8 bit tri-state output bus
    
);

    reg[7:0] count; //internal registers
    
    // Squential logic inside an always block
    
    always @(posedge clk or negedge rst_n) begin //triggers on rising edge of clock or falling edge of rst_n
    
        if (!rst_n) begin //(if the rst_n value is 0, this means a reset was triggered)
            count <= 8'b0;
        end else if (load) begin //Check control pin to see if the count value should be updated to data_in
            count <= data_in;
        end else if (en) begin //Increment counter by 1
            count <= count + 1'b1;
        end
    end
    
    assign data_out = oe ? count: 8'bZZZZZZZZ; //If output is enabled, set it to count, if it is not set it all tohigh impedance

endmodule