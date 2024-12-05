`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer:
//
// Create Date: 12/05/2024 12:33:42 PM
// Design Name:
// Module Name: top
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
module ram (
    input wire clk,              
    input wire [2:0] addr,       
    input wire [7:0] data_in,     
    input wire we,            
    output reg [7:0] data_out     
);

    reg [7:0] ram0, ram1, ram2, ram3, ram4, ram5, ram6, ram7;

    always @(posedge clk) begin
        if (we) begin
            case (addr)
                3'b000: ram0 <= data_in;
                3'b001: ram1 <= data_in;
                3'b010: ram2 <= data_in;
                3'b011: ram3 <= data_in;
                3'b100: ram4 <= data_in;
                3'b101: ram5 <= data_in;
                3'b110: ram6 <= data_in;
                3'b111: ram7 <= data_in;
            endcase
        end

        case (addr)
            3'b000: data_out <= ram0;
            3'b001: data_out <= ram1;
            3'b010: data_out <= ram2;
            3'b011: data_out <= ram3;
            3'b100: data_out <= ram4;
            3'b101: data_out <= ram5;
            3'b110: data_out <= ram6;
            3'b111: data_out <= ram7;
        endcase
    end
endmodule



module rom(
    input wire [2:0] addr,
    output reg [3:0] data_out
    );
   
    always @(*) begin
        case(addr)
            3'b000: data_out <= 4'd0;
            3'b001: data_out <= 4'd1;
            3'b010: data_out <= 4'd2;
            3'b011: data_out <= 4'd3;
            3'b100: data_out <= 4'd4;
            3'b101: data_out <= 4'd5;
            3'b110: data_out <= 4'd6;
            3'b111: data_out <= 4'd7;
        endcase
    end
endmodule


module RF(A, B, SA, SB, D, DA, W, rst, clk);
output [3:0]A; // A bus
output [3:0]B; // B bus
input SA; // Select A - A Address
input SB; // Select B - B Address
input [3:0]D; // Data input
input DA; // Data destination address
input W; // write enable
input rst; // positive logic asynchronous reset
input clk;

wire [1:0]load_enable;
wire [3:0]R00, R01;


Decoder1to2 decoder (load_enable, DA, W);
RegisterNbit reg00 (D,R00,load_enable[0], rst, clk); //D-in, R00-out
RegisterNbit reg01 (D,R01,load_enable[1], rst, clk);
Mux2to1Nbit muxA (A,R00, R01, SA);
Mux2to1Nbit muxB (B,R00, R01,SB);

endmodule

module RegisterNbit(D, Q,  L, R, clock);
parameter N = 8; // number of bits
output reg [N-1:0]Q; // registered output
input [N-1:0]D; // data input
input L; // load enable
input R; // positive logic asynchronous reset
input clock; // positive edge clock

always @(posedge clock or posedge R) begin
if(R)
Q <= 0;
else if(L)
Q <= D;
else
Q <= Q;
end
endmodule

module Decoder1to2(m, S, en);
input S; // select
input en; // enable (positive logic)
output [1:0]m; // 32 minterms

assign m[0] = ~S&en;
assign m[1] = S&en;

endmodule

module Mux2to1Nbit(o, i1,i2, s);
   input [3:0] i1,i2;
   input  s;
   output reg  [3:0] o;
 
always @(s or i1 or i2)
begin
   case (s)
      1'b0 : o = i1;
      1'b1 : o = i2;
      default : o = 4'b0;
   endcase
end
endmodule


module Multiplier(A, B, C);
    input [3:0] A, B;
    output [7:0] C;
   
    assign C = A * B;
endmodule





module ControlUnit(
    output reg [2:0] ROM_addr,
    output reg W_RF,
    output reg SA,
    output reg SB,
    output reg DA,
    output reg W_ram,
    input clk,
    input rst,
    input [2:0] Adr1_rom,
    input [2:0] Adr2_rom
    );

    reg [2:0] state;
    reg [2:0] next_state;
   
    parameter RF_write1 = 3'b000;
    parameter RF_write2 = 3'b001;
    parameter RF_read = 3'b010;
    parameter RF_mul = 3'b011;
    parameter RAM_write = 3'b100;
    parameter RAM_read = 3'b101;

    always @(posedge clk or posedge rst) begin
        if(rst) begin
            state <= RF_write1;
        end else begin
            state <= next_state;
        end

        case(state)
            RF_write1: begin
                next_state <= RF_write2;
                ROM_addr <= Adr1_rom;
                W_RF <= 1;
                SA <= 1;
                SB <= 0;
                DA <= 0;
                W_ram <= 0;
            end
            RF_write2: begin
                next_state <= RAM_write;
                ROM_addr <= Adr2_rom;
                W_RF <= 1;
                SA <= 1;
                SB <= 0;
                DA <= 1;
                W_ram <= 1;
            end
            RAM_write: begin
                next_state <= RF_write1;
                ROM_addr <= 0;
                W_RF <= 1;
                SA <= 0;
                SB <= 0;
                DA <= 0;
                W_ram <= 0;
            end
        endcase
    end
endmodule



module top(
    input clk,
    input rst,
    output [7:0] result,
    input [2:0] Adr1_rom,
    input [2:0] Adr2_rom,
    input [2:0] Adr_ram
    );

    wire [2:0] ROM_addr;
    wire [3:0] ROM_data_out;
    wire [7:0] RAM_out;
    wire W_RF, SA, SB, DA, W_ram;

    ControlUnit control_unit(
        .ROM_addr(ROM_addr),
        .W_RF(W_RF),
        .SA(SA),
        .SB(SB),
        .DA(DA),
        .W_ram(W_ram),
        .clk(clk),
        .rst(rst),
        .Adr1_rom(Adr1_rom),
        .Adr2_rom(Adr2_rom)
    );

    rom rom1(
        .addr(ROM_addr),
        .data_out(ROM_data_out)
    );

    wire [3:0] A, B;
    wire [7:0] C;

    RF rf(
        .A(A),
        .B(B),
        .SA(SA),
        .SB(SB),
        .D(ROM_data_out),
        .DA(DA),
        .W(W_RF),
        .rst(rst),
        .clk(clk)
    );

    Multiplier multiplier(
        .A(A),
        .B(B),
        .C(C)
    );

    ram ram1(
        .clk(clk),
        .addr(Adr_ram),
        .data_in(C),
        .we(W_ram),
        .data_out(RAM_out)
    );
   
    assign result = RAM_out;

endmodule

