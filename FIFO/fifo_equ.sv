module fifo_equ(
	input  logic                 clk,
  	input  logic                 rst,
  	input  logic                 writeEn,
  	input  logic [7:0] writeData,
  	input  logic                 readEn,
  	output logic [7:0] readData,
  	output logic                 full,
  	output logic                 empty
);
	logic o_full, o_empty;
	logic [7:0] o_readData;
	logic gold_full, gold_empty;
	logic [7:0] gold_readData;

	// original fifo
	fifo fifo_o( .clk(clk), .rst(rst), .in_read_ctrl(readEn), .in_write_ctrl(writeEn), .in_write_data(writeData), 
             .out_read_data(o_readData), .out_is_full(o_full), .out_is_empty(o_empty));

	fifo fifo( .clk(clk), .rst(rst), .in_read_ctrl(readEn), .in_write_ctrl(writeEn), .in_write_data(writeData), 
             .out_read_data(readData), .out_is_full(full), .out_is_empty(empty));

	// golden model fifo
	FIFO_gold gold(
 	.clk(clk), .rstN(rst), .writeEn(writeEn), .writeData(writeData), .readEn(readEn),
   .readData(gold_readData), .full(gold_full), .empty(gold_empty));
	
	//fifo_out_is_full: assume property (@(posedge clk) disable iff (rst) !( full && writeEn));
	//fifo_out_is_empty: assume property (@(posedge clk) disable iff (rst) !(empty && readEn));

	write_full_test: assert property (@(posedge clk) writeEn |->  o_full== gold_full);
	write_empty_test: assert property (@(posedge clk) writeEn |->  o_empty== gold_empty);
	read_full_test: assert property (@(posedge clk) readEn |->  o_full== gold_full);
	read_empty_test: assert property (@(posedge clk) readEn |->  o_empty== gold_empty);
	read_ability: assert property (@(posedge clk) readEn |->  o_readData== gold_readData);

	full_test: assert property (@(posedge clk) writeEn |->  o_full== full);
	empty_test: assert property (@(posedge clk) writeEn |->  o_empty== empty);
	read_full: assert property (@(posedge clk) readEn |->  o_full== full);
	read_empty: assert property (@(posedge clk) readEn |->  o_empty== empty);
	read_ability_test: assert property (@(posedge clk) readEn |->  o_readData== readData);



endmodule




module fifo( clk, rst, in_read_ctrl, in_write_ctrl, in_write_data, 
             out_read_data, out_is_full, out_is_empty
             );

   
parameter
  ENTRIES = 4; 
  
localparam [31:0]  
  ENTRIES_LOG2 = $clog2(ENTRIES);
  
   input  logic       clk; 
   input  logic       rst;
   input  logic       in_read_ctrl;
   input  logic       in_write_ctrl;
   input  logic [7:0] in_write_data;
   output logic [7:0] out_read_data;
   output logic       out_is_full;
   output logic       out_is_empty;

   logic [ENTRIES_LOG2-1:0]  write_ptr;
   logic [ENTRIES_LOG2-1:0]  read_ptr;
   logic [ENTRIES-1:0] [7:0] fifo_data;
   logic [7:0]               head;
   logic [ENTRIES_LOG2:0]    number_of_current_entries; 

//fifo_cover_ent_0: cover property (@(posedge clk) disable iff(rst) number_of_current_entries == 0);
//fifo_cover_ent_1: cover property (@(posedge clk) disable iff(rst) number_of_current_entries == 1);
//fifo_cover_ent_2: cover property (@(posedge clk) disable iff(rst) number_of_current_entries == 2);
//fifo_cover_ent_3: cover property (@(posedge clk) disable iff(rst) number_of_current_entries == 3);
//fifo_cover_ent_4: cover property (@(posedge clk) disable iff(rst) number_of_current_entries == 4);
//fifo_cover_ent_5: cover property (@(posedge clk) disable iff(rst) number_of_current_entries == 5);
//fifo_cover_ent_6: cover property (@(posedge clk) disable iff(rst) number_of_current_entries == 6);
//fifo_cover_ent_7: cover property (@(posedge clk) disable iff(rst) number_of_current_entries == 7);
//fifo_cover_ent_8: cover property (@(posedge clk) disable iff(rst) number_of_current_entries == 8);

//fifo_out_is_full: assume property (@(posedge clk) disable iff (rst) !(out_is_full && in_write_ctrl));
//fifo_out_is_empty: assume property (@(posedge clk) disable iff (rst) !(out_is_empty && in_read_ctrl));
   
always_ff @(posedge clk) begin
   if (rst) begin
      write_ptr <= 0;
   end
   else if (in_write_ctrl) begin
      write_ptr <= write_ptr + 1'b1;
      fifo_data[write_ptr] <= in_write_data;
   end
end

always_comb begin
   head = fifo_data[read_ptr];
end
   
always_ff @(posedge clk) begin
   if (rst) begin
      read_ptr <= 0;
   end
   else if (in_read_ctrl) begin
      read_ptr <= read_ptr + 1'b1;
      out_read_data <= head;
   end
end

always_ff @(posedge clk) begin
   if (rst) begin
      number_of_current_entries <= 0;
      out_is_empty <= 1;
      out_is_full <= 0;
   end
   else if (in_read_ctrl & ~in_write_ctrl) begin
//	if(out_is_empty == 0) begin
	      number_of_current_entries <= number_of_current_entries - 1'b1;
	      out_is_full <= 0;
	      out_is_empty <= (number_of_current_entries == 1'b1);
//	end       
   end
   else if (~in_read_ctrl & in_write_ctrl & ~out_is_full) begin
//	if(out_is_full == 0)begin
	      number_of_current_entries <= number_of_current_entries + 1'b1;
	      out_is_empty <= 0;
	      out_is_full <= (number_of_current_entries == (ENTRIES-1'b1)); 
//	end
   end
//    else if (~in_read_ctrl & ~in_write_ctrl) begin
//      out_is_empty <= 0;
//     out_is_full <= 0; 
//  end
end
   
endmodule



module FIFO_gold #(
  parameter  DataWidth = 8,
  parameter  Depth     = 4,
  localparam PtrWidth  = $clog2(Depth)
) (
  input  logic                 clk,
  input  logic                 rstN,
  input  logic                 writeEn,
  input  logic [DataWidth-1:0] writeData,
  input  logic                 readEn,
  output logic [DataWidth-1:0] readData,
  output logic                 full,
  output logic                 empty
);

  logic [DataWidth-1:0] mem[Depth];
  logic [PtrWidth:0] wrPtr, wrPtrNext;
  logic [PtrWidth:0] rdPtr, rdPtrNext;

  always_comb begin
    wrPtrNext = wrPtr;
    rdPtrNext = rdPtr;
    if (writeEn) begin
      wrPtrNext = wrPtr + 1;
    end
    if (readEn) begin
      rdPtrNext = rdPtr + 1;
    end
  end

  always_ff @(posedge clk or negedge rstN) begin
    if (rstN) begin
      wrPtr <= '0;
      rdPtr <= '0;
    end else begin
      wrPtr <= wrPtrNext;
      rdPtr <= rdPtrNext;
    end

    mem[wrPtr[PtrWidth-1:0]] <= writeData;
  end

  assign readData = mem[rdPtr[PtrWidth-1:0]];

  assign empty = (wrPtr[PtrWidth] == rdPtr[PtrWidth]) && (wrPtr[PtrWidth-1:0] == rdPtr[PtrWidth-1:0]);
  assign full  = (wrPtr[PtrWidth] != rdPtr[PtrWidth]) && (wrPtr[PtrWidth-1:0] == rdPtr[PtrWidth-1:0]);

endmodule
