`timescale 1ns / 1ps
`include "ovl_zero_one_hot.v"
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


module datapath(
	input logic clk,
	input logic rst,
	input logic clear,
	input logic enable, 
    input logic [8:0] insF,
    input logic [31:0] WritetE, //from Enable stage
    
    input logic [31:0] ImemD, //from memory
    input logic signed [31:0] SrcBE, //from input
    input logic signed [31:0] writeData, imemData, Data, 
 //input logic [31:0] SrcAE,
    output logic  [31:0] ALUResultM, 
			output logic Zero, Sign
    );

	
    logic [3:0] ALUControl;
    logic opb5, funct7b5;
	logic [2:0] funct3;
	logic [1:0] ALUOP;
	logic Overflow;
	logic [31:0] ALU_o;
	logic [1:0] mux_select;
	logic [31:0] SrcAE;
	logic [8:0] ins;
	logic [31:0] ALUResult; //from input
	logic [31:0] ALUResult_ref;
	logic [31:0] ALUResultE;
	logic Zero_ref, Sign_ref, Overflow_ref;
	
    assign ALU_o = SrcAE+SrcBE;
    assign overflow = (SrcAE[31] & SrcBE[31] & ~ALU_o[31]) | (~SrcAE[31] & ~SrcBE[31] & ALU_o[31]);

    assign mux_select = ins[8:7];	
    assign opb5 = ins[6];
    assign funct7b5 = ins[5];
    assign funct3 = ins[4:2];
    assign ALUOP = ins[1:0];
    wire [2:0] fire_mux;

    //Instruction Fetch
    IF_ID IF(.clk(clk), .reset(rst), .clear(clear), .enable(enable), .InstrF(insF),  .InstrD(ins));
    // Mux
    mux3 MUX3(.d0(writeData), .d1(imemData), .d2(Data), .s(mux_select), .y(SrcAE));

    // ALU
    aludec Decoder(.opb5(opb5), .funct3(funct3), .funct7b5(funct7b5), .ALUOp(ALUOP), .ALUControl(ALUControl));
    alu ALU(.SrcA(SrcAE), .SrcB(SrcBE), .ALUControl(ALUControl), .ALUResult(ALUResult), .Zero(Zero), .Sign(Sign), .Overflow(Overflow));
    alu_reference ALU_ref(.SrcA(SrcAE), .SrcB(SrcBE), .ALUControl(ALUOP), .ALUResult(ALUResult_ref), .Zero(Zero_ref), .Sign(Sign_ref), .Overflow(Overflow_ref));
    
    //Execution
    IEx Execution(.clk(clk), .reset(rst), .ALUResult(ALUResult), .ALUResultE(ALUResultE));

    //Memory
    IEx_IMem Memory(.clk(clk), .reset(rst), .ALUResultE(ALUResultE), .ALUResultM(ALUResultM));
    

    // instruction test
    ins_stability : assert property (@(posedge clk) ins |-> (ALUControl>=0 & ALUControl <=15));
    genvar j;
	generate for (j=0; j<=2; j++)begin: ins_latency_checking
    	    ins_latency_test : assert property (@(posedge clk) ((clear&enable) |-> ##j (IF.InstrD == 0)));
	end
	endgenerate


    // Mux checking
    ovl_zero_one_hot #(.width(2))
	ovl_zero_one_hot_check (
		.clock(clk), .reset(!rst),
		.enable(1'b1), .test_expr(mux_select), .fire(fire_mux));
    ovl_check: assume property (@(posedge clk) insF[8:7] != 11);

    //alu checking
	genvar m;
	generate for (m=0; m<=20; m++)begin: latency_checking
    	    valid_latency0 : cover property (@(posedge clk) ((~clear&enable) |-> ##m (ALUResultM)));
	end
	endgenerate


    	no_overflow: assert property (@(posedge clk) (SrcAE[31] ^ SrcBE[31]) |-> ~Overflow);
    	overflow_verify: assert property (@(posedge clk) Overflow == overflow);
    	overflow_checking: assert property (@(posedge clk) (SrcAE[31] ^ SrcBE[31]) |-> ~overflow);


    	// reference code verification of add and subtration
    	arithemic_add_verify: assert property (@(posedge clk) (ALUOP == 00) |->  ALUResult_ref== ALU.ALUResult);
    	arithemic_sub_verify: assert property (@(posedge clk) (ALUOP == 01) |->  ALUResult_ref== ALU.ALUResult);
    	arithemic_and_verify: assert property (@(posedge clk) (ins[4:0] == 11111) |->  ALUResult_ref== ALUResult);
    	arithemic_or_verify: assert property (@(posedge clk) (ins[4:0] == 11010) |->  ALUResult_ref== ALUResult);



        // liveness property
	liveness_checking_ALU : assert property (@(posedge clk) ((ins) |-> s_eventually (ALU.ALUResult | Zero)));

    	liveness_checking_ALU_dis : assert property (@(posedge clk) disable iff (Zero) ((ins) |-> s_eventually (ALU.ALUResult)));

	genvar E;
	generate for (E=0; E<=5; E++)begin: execution_latency_checking
    	    letency_execution : assert property (@(posedge clk) disable iff (rst&~Zero) (ALUResult==1 |-> ##E ALUResultE==1));
	end
	endgenerate

	liveness_checking_datapath : assert property (@(posedge clk) ((!clear&enable) |-> s_eventually (ALUResultM | Zero)));
	
	liveness_checking_ALU_execution : assert property (@(posedge clk) disable iff (rst&~Zero) ALU.ALUResult |-> ##2 (Execution.ALUResultE == ALU.ALUResult));

        liveness_checking_ALU_Memory : assert property (@(posedge clk) disable iff (rst&~Zero) (ins |-> s_eventually (ALUResultM)));

	genvar M;
	generate for (M=0; M<=5; M++)begin: Memory_latency_checking
    	    letency_execution : assert property (@(posedge clk) disable iff (rst&~Zero) (ALUResultE == 1 |-> ##M ALUResultM == 1));
	end
	endgenerate

	
	
    
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
			//default: ALUControl = 4'bxxxx; // ???
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
			output logic Zero, Sign, Overflow);

logic [31:0] Sum;

assign Sum = SrcA + (ALUControl[0] ? ~SrcB : SrcB) + ALUControl[0];  // sub using 1's complement
//assign Sum = SrcA + SrcB;
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

module IEx_IMem(input logic clk, reset,
                input logic [31:0] ALUResultE, 
                output logic [31:0] ALUResultM);

always_ff @( posedge clk, posedge reset ) begin 
    if (reset) begin
        ALUResultM <= 0;
    end

    else begin
        ALUResultM <= ALUResultE;       
    end
    
end

endmodule
/*
module IEx_IMem(input logic clk, reset,
                input logic [31:0] ALUResultE, 
                output logic [31:0] ALUResultM);

always_ff @( posedge clk, posedge reset ) begin 
    if (reset) begin
        ALUResultM <= 0;
    end

    else begin
        ALUResultM <= ALUResultE;       
    end
    
end

endmodule
*/

module alu_reference(input logic [31:0] SrcA, 
			input logic [31:0] SrcB, 
			input logic [1:0] ALUControl , 
			output logic  [31:0] ALUResult, 
			output logic Zero, Sign, Overflow);

//logic [31:0] Sum;

//assign Sum = SrcA + (ALUControl[0] ? ~SrcB : SrcB) + ALUControl[0];  // sub using 1's complement
//assign Overflow = ~(ALUControl[0] ^ SrcB[31] ^ SrcA[31]) & (SrcA[31] ^ Sum[31]) & (~ALUControl[1]);
assign Overflow = (SrcA[31] & SrcB[31] & ~ALUResult[31]) | (~SrcA[31] & ~SrcB[31] & ALUResult[31]);

assign Zero = ~(|ALUResult);
assign Sign = ALUResult[31];


always_comb
		casex (ALUControl)
				2'b00: ALUResult = SrcA + SrcB;	
				2'b01: ALUResult = SrcA - SrcB;	// and
				2'b10: ALUResult = SrcA | SrcB;	// or
				2'b11: ALUResult = SrcA & SrcB;	// sll, slli
				default: ALUResult = 32'bx;
		endcase


endmodule

module mux3 (input logic [31:0] d0, d1, d2,input logic [1:0] s,output logic [31:0] y);
	assign y = s[1] ? d2 : (s[0] ? d1 : d0);
endmodule


module IF_ID (input logic clk, reset, clear, enable,
            input logic [7:0] InstrF,
            output logic [7:0] InstrD);


always_ff @( posedge clk, posedge reset ) begin
    if (reset) begin // Asynchronous Clear
        InstrD <= 0;
    end

    else if (enable) begin 
		 if (clear) begin // Synchrnous Clear
			  InstrD <= 0;	 
		 end
		 
		 else begin	 
			  InstrD <= InstrF;
		 end
	 end
end

endmodule

module IEx(input logic clk, reset,
                input logic [31:0] ALUResult, 
                output logic [31:0] ALUResultE);

always_ff @( posedge clk, posedge reset ) begin 
    if (reset) begin
        ALUResultE <= 0;
    end

    else begin
        ALUResultE <= ALUResult;       
    end
    
end

endmodule
