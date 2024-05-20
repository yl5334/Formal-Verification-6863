`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12/15/2023 01:42:37 AM
// Design Name: 
// Module Name: ALU_unit
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


module ALU_unit( 
    input logic [6:0] ins,
    input logic [31:0] WritetE, //from Enable stage
    input logic [31:0] ALUresult, //from input
    input logic [31:0] ImemD, //from memory
    input logic [31:0] SrcBE, //from input
    input logic [31:0] SrcAE,
    output logic  [31:0] ALUResult, 
			output logic Zero, Sign
    );
    logic [3:0] ALUControl;
    logic opb5, funct7b5;
	logic [2:0] funct3;
	logic [1:0] ALUOp;
	
    assign opb5 = ins[6];
    assign funct7b5 = ins[5];
    assign funct3 = ins[4:2];
    assign ALUOP = ins[1:0];

	valid_latency : assert property (@posedge clk) ((ins) |-> ##2 ALUResult);

    aludec decoder(opb5, funct3, funct7b5, ALUOp, ALUControl);
    alu alu(SrcAE, SrcBE, ALUControl, ALUResult, Zero, Sign);
    
endmodule

/*
	Name: ALU Decoder
	Description: Receives control signal from the Main Decoder Unit and 
	determines the type of operation that has to be performed by the ALU

*/



module aludec(input logic opb5,
	input logic [2:0] funct3,
	input logic funct7b5,
	input logic [1:0] ALUOp,
	output logic [3:0] ALUControl);
	
logic RtypeSub;
assign RtypeSub = funct7b5 & opb5; // TRUE for R-type subtract
always_comb
	case(ALUOp)
		2'b00: ALUControl = 4'b0000; // addition
		2'b01: ALUControl = 4'b0001; // subtraction
		default: case(funct3) // R-type or I-type ALU
			3'b000: if (RtypeSub)
				ALUControl = 4'b0001; // sub
			else
				ALUControl = 4'b0000; // add, addi
			3'b001: ALUControl = 4'b0100; // sll, slli
			3'b010: ALUControl = 4'b0101; // slt, slti
			3'b011: ALUControl = 4'b1000; // sltu, sltiu
			3'b100: ALUControl = 4'b0110; // xor, xori
			3'b101: if (~funct7b5)
				ALUControl = 4'b0111;	// srl
			else
				ALUControl = 4'b1111;  // sra
			3'b110: ALUControl = 4'b0011; // or, ori
			3'b111: ALUControl = 4'b0010; // and, andi
			default: ALUControl = 4'bxxxx; // ???
			endcase
	endcase
	
endmodule

/*
	Name: ALU Unit
	Description: Receives control signals from the ALU Decoder and performs the operations
*/


module alu(input logic [31:0] SrcA, 
			input logic [31:0] SrcB, 
			input logic [3:0] ALUControl , 
			output logic  [31:0] ALUResult, 
			output logic Zero, Sign);

logic [31:0] Sum;
logic Overflow;

assign Sum = SrcA + (ALUControl[0] ? ~SrcB : SrcB) + ALUControl[0];  // sub using 1's complement
assign Overflow = ~(ALUControl[0] ^ SrcB[31] ^ SrcA[31]) & (SrcA[31] ^ Sum[31]) & (~ALUControl[1]);

assign Zero = ~(|ALUResult);
assign Sign = ALUResult[31];


always_comb
		casex (ALUControl)
				4'b000x: ALUResult = Sum;				// sum or diff
				4'b0010: ALUResult = SrcA & SrcB;	// and
				4'b0011: ALUResult = SrcA | SrcB;	// or
				4'b0100: ALUResult = SrcA << SrcB;	// sll, slli
				4'b0101: ALUResult = {{30{1'b0}}, Overflow ^ Sum[31]}; //slt, slti
				4'b0110: ALUResult = SrcA ^ SrcB;   // Xor
				4'b0111: ALUResult = SrcA >> SrcB;  // shift logic
				4'b1000: ALUResult = ($unsigned(SrcA) < $unsigned(SrcB)); //sltu, stlui
				4'b1111: ALUResult = SrcA >>> SrcB; //shift arithmetic
				default: ALUResult = 32'bx;
		endcase


endmodule

//module mux3 (input logic [31:0] d0, d1, d2,input logic [1:0] s,output logic [31:0] y);
//	assign y = s[1] ? d2 : (s[0] ? d1 : d0);
//endmodule
