`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 12/05/2024 03:16:17 PM
// Design Name:
// Module Name: rom_tb
// Project Name:
// Target Devices:
// Tool Versions:
// Description:
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////


module rom_tb;

    reg [2:0] addr;            
    wire [3:0] data_out;  

    rom uut (
        .addr(addr),
        .data_out(data_out)
    );

    initial begin
     
        addr = 3'b000;
        #10;

        addr = 3'b001;
        #10;
       
        addr = 3'b010;
        #10;
       
        addr = 3'b011;
        #10;
       
        addr = 3'b100;
        #10;
       
        addr = 3'b101;
        #10;
       
        addr = 3'b110;
        #10;
       
        addr = 3'b111;
        #10;
       
        $stop;
    end

endmodule
