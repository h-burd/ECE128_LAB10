`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 12/05/2024 12:56:58 PM
// Design Name:
// Module Name: top_tb
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

module top_tb();


reg clk, rst;
wire [7:0] result;
reg [2:0] Adr1_rom, Adr2_rom, Adr_ram;

top uut(
    .clk(clk),
    .rst(rst),
    .result(result),
    .Adr1_rom(Adr1_rom),
    .Adr2_rom(Adr2_rom),
    .Adr_ram(Adr_ram)
);

initial begin
    clk = 0;
    forever #5 clk = ~clk;
end

initial begin
    rst = 1;
    #10;
    rst = 0;
    Adr1_rom = 3'd1;
    Adr2_rom = 3'd1;
    Adr_ram = 3'd0;
    #100;
    Adr1_rom = 3'd1;
    Adr2_rom = 3'd2;
    Adr_ram = 3'd1;
    #100;
    Adr1_rom = 3'd2;
    Adr2_rom = 3'd3;
    Adr_ram = 3'd2;
    #100;
    Adr1_rom = 3'd3;
    Adr2_rom = 3'd4;
    Adr_ram = 3'd3;
    #100;
    Adr1_rom = 3'd4;
    Adr2_rom = 3'd5;
    Adr_ram = 3'd4;
    #100;
    Adr1_rom = 3'd5;
    Adr2_rom = 3'd6;
    Adr_ram = 3'd5;
    #100;
    $stop;
end


endmodule
