`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 12/05/2024 03:10:21 PM
// Design Name:
// Module Name: ram_tb
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

module ram_tb;

    reg clk;
    reg [2:0] addr;
    reg [7:0] data_in;
    reg we;
    wire [7:0] data_out;

    ram uut (
        .clk(clk),
        .addr(addr),
        .data_in(data_in),
        .we(we),
        .data_out(data_out)
    );

    always begin
        #5 clk = ~clk;
    end


    initial begin
        clk = 0;
        addr = 3'b000;
        data_in = 8'h00;
        we = 0;
        #10;
        addr = 3'b000;
        data_in = 8'h12;  
        we = 1;
        #10;
        we = 0;          
        #10;
        addr = 3'b000;
        #20;
        addr = 3'b011;
        data_in = 8'h34; 
        we = 1;          
        #10;
        we = 0;         
        #10;
        addr = 3'b011;
        #20;
        addr = 3'b111;
        data_in = 8'h56;  
        we = 1;           
        #10;
        we = 0;           
        #10;
        addr = 3'b111;
        #20;
        $stop;
    end

endmodule
