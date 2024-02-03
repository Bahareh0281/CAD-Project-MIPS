/*
DESCRIPTION
Add Module

NOTES

TODO

*/

module add_r0 #(
	parameter DATA_WIDTH = 32
)(
	input [DATA_WIDTH - 1:0] input1,
	input [DATA_WIDTH - 1:0] input2,
	output[DATA_WIDTH - 1:0] dataOut,
	output C,
	output Z,
	output V,
	output S
);
/**********
 *  Array Packing Defines 
 **********/
/**********
 * Internal Signals
**********/
reg [DATA_WIDTH:0] tmpAdd;
reg Ctmp, Ztmp, Vtmp, Stmp;
/**********
 * Glue Logic 
 **********/
/**********
 * Synchronous Logic
 **********/
/**********
 * Glue Logic 
 **********/
/**********
 * Components
 **********/
/**********
 * Output Combinatorial Logic
 **********/
 always @(input1, input2) begin
	Ctmp = 0;
	Ztmp = 0;
	Vtmp = 0;
	Stmp = 0;
	
	tmpAdd = input1 + input2;
	
	Ctmp = tmpAdd[DATA_WIDTH];	// Carry Flag
	
	if(tmpAdd[DATA_WIDTH-1:0] == {(DATA_WIDTH){1'b0}}) begin
		Ztmp = 1;
	end
	
	if((input1[DATA_WIDTH - 1] == input2[DATA_WIDTH - 1]) && (tmpAdd[DATA_WIDTH - 1] != input1[DATA_WIDTH - 1])) begin
		Vtmp = 1;
	end

	Stmp = tmpAdd[DATA_WIDTH - 1];
 end
 
 assign dataOut = tmpAdd[DATA_WIDTH - 1:0];
 assign C = Ctmp;
 assign Z = Ztmp;
 assign V = Vtmp;
 assign S = Stmp;
 
endmodule


/*
DESCRIPTION
ALU Controller Module

NOTES

TODO
*/

module alu_controller_r0 (
	input [2:0] ALUOp,		// ALUOp from the main controller
	input [5:0] funcode,	// Function code from instruction
	output [4:0] ALUCtrl,	// ALU function
	output JumpReg
);

/**********
 * Internal Signals
**********/
reg [4:0] ALUCtrl_tmp;
reg JumpReg_tmp;

/**********
 * Glue Logic 
 **********/
 // ALUOp Signals
 localparam ALUadd = 3'b000;
 localparam ALUsub = 3'b001;
 localparam ALUand = 3'b010;
 localparam ALUor  = 3'b011;
 localparam ALUxor = 3'b100;
 localparam ALUslt = 3'b101;
 localparam ALURtp = 3'b111;
 
 // Function codes
 localparam fun_sll		= 6'h00;
 localparam fun_srl		= 6'h02;
 localparam fun_sra		= 6'h03;
 localparam fun_sllv	= 6'h04;
 localparam fun_srlv	= 6'h06;
 localparam fun_srav	= 6'h07;
 localparam fun_jr		= 6'h08;
 //localparam fun_jalr	= 6'h09;
 localparam fun_mfhi	= 6'h10;
 localparam fun_mthi	= 6'h11;
 localparam fun_mflo	= 6'h12;
 localparam fun_mtlo	= 6'h13;
 localparam fun_mult	= 6'h18;
 localparam fun_multu	= 6'h19;
 localparam fun_div		= 6'h1A;
 localparam fun_divu	= 6'h1B;
 localparam fun_add 	= 6'h20;
 localparam fun_addu 	= 6'h21;
 localparam fun_sub		= 6'h22;
 localparam fun_subu	= 6'h23;
 localparam fun_and		= 6'h24;
 localparam fun_or		= 6'h25;
 localparam fun_xor		= 6'h26;
 localparam fun_nor		= 6'h27;
 localparam fun_slt		= 6'h2A;
 localparam fun_sltu	= 6'h2B;
 
/**********
 * Synchronous Logic
 **********/
/**********
 * Glue Logic 
 **********/
/**********
 * Components
 **********/
/**********
 * Output Combinatorial Logic
 **********/
 always @(ALUOp, funcode) begin
	JumpReg_tmp = 0;
 
	//----------------------- NOT R-Type ---------------------------//
	if(ALUOp == ALUadd) begin
		ALUCtrl_tmp <= 5'b00000;
	end else if(ALUOp == ALUsub) begin
		ALUCtrl_tmp <= 5'b00001;	
	end else if(ALUOp == ALUand) begin
		ALUCtrl_tmp <= 5'b01101;
	end else if(ALUOp == ALUor) begin
		ALUCtrl_tmp <= 5'b01110;
	end else if(ALUOp == ALUxor) begin
		ALUCtrl_tmp <= 5'b01111;
	end else if(ALUOp == ALUslt) begin
		ALUCtrl_tmp <= 5'b10001;	
	
	//------------------------- If R-Type --------------------------//
	end else if((funcode == fun_add) || (funcode == fun_addu)) begin
		ALUCtrl_tmp <= 5'b00000;
	end else if((funcode == fun_sub) || (funcode == fun_subu)) begin
		ALUCtrl_tmp <= 5'b00001;
	end else if((funcode == fun_mult) || (funcode == fun_multu)) begin
		ALUCtrl_tmp <= 5'b00010;
	end else if(funcode == fun_sll) begin
		ALUCtrl_tmp <= 5'b00011;
	end else if(funcode == fun_sllv) begin
		ALUCtrl_tmp <= 5'b00100;
	end else if(funcode == fun_srl) begin
		ALUCtrl_tmp <= 5'b00101;
	end else if(funcode == fun_srlv) begin
		ALUCtrl_tmp <= 5'b00110;
	end else if(funcode == fun_sra) begin
		ALUCtrl_tmp <= 5'b00111;
	end else if(funcode == fun_srav) begin
		ALUCtrl_tmp <= 5'b01000;
	end else if(funcode == fun_mfhi) begin
		ALUCtrl_tmp <= 5'b01001;
	end	else if(funcode == fun_mflo) begin
		ALUCtrl_tmp <= 5'b01010;
	end else if(funcode == fun_mthi) begin
		ALUCtrl_tmp <= 5'b01011;
	end else if(funcode == fun_mtlo) begin
		ALUCtrl_tmp <= 5'b01100;
	end else if(funcode == fun_and) begin
		ALUCtrl_tmp <= 5'b01101;
	end else if(funcode == fun_or) begin
		ALUCtrl_tmp <= 5'b01110;
	end else if(funcode == fun_xor) begin
		ALUCtrl_tmp <= 5'b01111;
	end else if(funcode == fun_nor) begin
		ALUCtrl_tmp <= 5'b10000;
	end else if(funcode == fun_slt) begin
		ALUCtrl_tmp <= 5'b10001;
	end else if(funcode == fun_sltu) begin
		ALUCtrl_tmp <= 5'b10010;
	end else if(funcode == fun_jr) begin
		ALUCtrl_tmp <= 5'b00000;
		JumpReg_tmp = 1;
	end
 end
 
 assign ALUCtrl = ALUCtrl_tmp;
 assign JumpReg = JumpReg_tmp;
 
endmodule



/*
DESCRIPTION:
Comparator For Branching in the ID stage
Checks if the inputs are equivalent and sets the "equal" flag accordingly

NOTES:
TODO:
*/

module comparator_r0 #(
	parameter BIT_WIDTH = 32
)(
	input [2*BIT_WIDTH - 1:0] dataIn,
	output equal
);

/**********
 * Internal Signals
 **********/
 reg equal_tmp;
	 
/**********
 * Glue Logic 
 **********/
/**********
 * Synchronous Logic
 **********/
/**********
 * Glue Logic 
 **********/
/**********
 * Components
 **********/
/**********
 * Output Combinatorial Logic
 **********/
	always @(dataIn) begin
		if(dataIn[2*BIT_WIDTH - 1:BIT_WIDTH] == dataIn[BIT_WIDTH - 1:0]) begin
			equal_tmp <= 1'b1;
		end else begin
			equal_tmp <= 1'b0;
		end
	end

	assign equal = equal_tmp;
	
endmodule



/*
 * DESCRIPTION
 * This creates an array or vector registers of generic bitwidth, array width, and pipeline depth. 
 * 
 * NOTES
 * BIT_WIDTH = bitwidth
 * DEPTH = input/output array width
 * DELAY = pipeline depth 
 * clk = clock
 * rst = synchronous, active high reset, clears out pipeline
 * en_n = active low enable
 * dataIn = vectorized input array
 * dataOut = vectorized output array
 * 
 * TODO
 * 
 */


 module delay_r0 #(
 	parameter BIT_WIDTH = 4,
 	parameter DEPTH = 2,
 	parameter DELAY = 4
 )(
 	input clk,
 	input rst,
 	input en_n,
 	input [BIT_WIDTH*DEPTH - 1:0] dataIn,
 	output [BIT_WIDTH*DEPTH - 1:0] dataOut
 );
 	/**********
 	 *  Array Packing Defines 
 	 **********/
 //These are preprocessor defines similar to C/C++ preprocessor or VHDL functions
 	`define PACK_ARRAY(PK_WIDTH,PK_DEPTH,PK_SRC,PK_DEST, BLOCK_ID, GEN_VAR)    genvar GEN_VAR; generate for (GEN_VAR=0; GEN_VAR<(PK_DEPTH); GEN_VAR=GEN_VAR+1) begin: BLOCK_ID assign PK_DEST[((PK_WIDTH)*GEN_VAR+((PK_WIDTH)-1)):((PK_WIDTH)*GEN_VAR)] = PK_SRC[GEN_VAR][((PK_WIDTH)-1):0]; end endgenerate
 	`define UNPACK_ARRAY(PK_WIDTH,PK_DEPTH,PK_DEST,PK_SRC, BLOCK_ID, GEN_VAR)  genvar GEN_VAR; generate for (GEN_VAR=0; GEN_VAR<(PK_DEPTH); GEN_VAR=GEN_VAR+1) begin: BLOCK_ID assign PK_DEST[GEN_VAR][((PK_WIDTH)-1):0] = PK_SRC[((PK_WIDTH)*GEN_VAR+(PK_WIDTH-1)):((PK_WIDTH)*GEN_VAR)]; end endgenerate

 	/**********
 	 * Internal Signals
 	 **********/
 //These iterators are purely use for for-loops which get unrolled in synthesis
 //So we can use integers here since these do not become hardware
 	integer i,j; //iterators
	
 //Wire data type is used for nonregistered signals
 //assign with "assign wire_signal_name = value_for_wire_signal;"
 	wire [BIT_WIDTH - 1:0] tmp [DEPTH - 1:0]; //input as array
 	wire [BIT_WIDTH*DEPTH - 1:0] tmpOut; //wire for output, more of this at end
	
 //NOTE: in VHDL, there are for generate and if generate statements
 //this is done in verilog with "generate"
 //packing and unpacking array defines are examples of for-generate
 //in the "generate" blocks, in addition to instantiations, one can do assignments 
 //to wire (becuase nonsynchronous), which is done in the packing and unpacking defines	
	
 //Register data type is used for synchronous logic
 //assign with "<="
 	reg [BIT_WIDTH - 1:0] pipe [DELAY-1:0][DEPTH - 1:0]; //data pipeline

 	/**********
 	 * Glue Logic 
 	 **********/
 //very often there needs to be some logic/decoding to inputs to a module 
 //here we just unpacking the input vector into an array 
 	`UNPACK_ARRAY(BIT_WIDTH,DEPTH,tmp,dataIn, U_BLK_0, idx_0)
	
 	/**********
 	 * Synchronous Logic
 	 **********/
 //NOTE in C/C++ "{" and "}" are used to show hierarchy, in verilog use "begin" and "end"
	 
 //similar to vhdl process statements, verilog has always blocks 
 //"posedge clk" is the sensitivity list
 //since the reset is not in the sensitivity list, the reset is synchronous
 	always @(posedge clk) begin
		
 		// Synchronous Reset
 //"1'b0" translates to "one bit with value binary 0"  
 		if(rst == 1'b1)	begin
 			//reset all the stages of the pipeline to zero
 			for(j=0; j<DELAY; j=j+1) begin //For all delay layers
 				for(i=0; i<DEPTH; i=i+1) begin //For all depth of input array
 					pipe[j][i] <= {(BIT_WIDTH){1'b0}};
 				end
 			end
 		end

 		else begin
 //all for-loops are unrolled, so should never have sequential dependences				
 //since all loops are unrolled and no sequential dependences, 
 //all assignments occure at the same time (i.e. clock edge)

 			// Pipeline delay
 			if(en_n == 1'b0) begin			
 				for(i=0;i<DEPTH; i=i+1) begin //For all depth of input array
 					pipe[0][i] <= tmp[i]; 
 				end
 			end

 //only got to DELAY - 1 since the last stage of pipe is overwritten and zeroth stage assigned above
 			for(i=0; i<DELAY-1; i=i+1) begin 
 				for(j=0;j<DEPTH; j=j+1) begin //For all depth of input array
 					pipe[i+1][j] <= pipe[i][j]; //Pipe shifts makes delay
 				end
 			end
 		end
 	end
				
 	/**********
 	 * Glue Logic 
 	 **********/
 	/**********
 	 * Components
 	 **********/
 	/**********
 	 * Output Combinatorial Logic
 	 **********/
 //currently pipe is a 2D array of logic vectors, so we need to make the last delay stage into a vector
 	`PACK_ARRAY(BIT_WIDTH,DEPTH,pipe[(DELAY-1)],tmpOut,U_BLK_1,idx_1)
	 
 //to be able to do conditional combinatorial logic, use "generate"
 	//When delay is 0, this entity should be a wire
 	generate
 	if(DELAY > 0)
 		assign dataOut = tmpOut;
		
 //NOTE the below statement would be ideal rather than having another temp signal but this isnt allowed since
 //pack array also uses a generate statement and they can not be nested
 //'PACK_ARRAY(BIT_WIDTH,DEPTH,dataOut,pipe[DELAY-1],U_BLK_1,idx_1)

 	else
 		assign dataOut = dataIn;
 	endgenerate
	
 endmodule


 /*
 DESCRIPTION:
 General Delay Wrapper Entity

 NOTES:
 This module is to be called by other modules for a functional entity.
 This module wraps various implementations.

 TODO:

 */
  
 

 module delay #(
 	parameter BIT_WIDTH = 32,
 	parameter DEPTH = 1,
 	parameter DELAY = 1,
 	parameter ARCH_SEL = 0
 )(
 	input clk,
 	input rst,
 	input en_n,
 	input [BIT_WIDTH*DEPTH-1:0] dataIn,
 	output [BIT_WIDTH*DEPTH-1:0] dataOut
 );
	
 	/**********
 	 * Internal Signals
 	 **********/
 	/**********
 	 * Glue Logic 
 	 **********/
 	/**********
 	 * Synchronous Logic
 	 **********/
 	/**********
 	 * Glue Logic 
 	 **********/
 	/**********
 	 * Components
 	 **********/
 //generate if(!ARCH_SEL) //since only have a single implementation, dont need now
 	delay_r0 #(
 		.BIT_WIDTH(BIT_WIDTH),
 		.DEPTH(DEPTH),
 		.DELAY(DELAY)
 	)U_IP(
 		.clk(clk),
 		.rst(rst),
 		.en_n(en_n),
 		.dataIn(dataIn),
 		.dataOut(dataOut)
 	);
 //endgenerate	

 	/**********
 	 * Output Combinatorial Logic
 	 **********/


 endmodule



/*
DESCRIPTION
Counter

NOTES
	MAX_COUNT		// Max count value
	COUNT_WIDTH 	// Bit width of the count
	
	clk				// Clock
	rst				// Synchronous reset
	load			// Whether to load the input value or not
	dataIn			// Value to load into the counter
	count			// Count value

TODO

*/

module counter_r0 #(
	parameter MAX_COUNT = 4,
	parameter COUNT_WIDTH = log2(MAX_COUNT) + 1,
	parameter DELAY = 0
)(
	input clk,
	input rst,
	input load,
	input pause,
	input [COUNT_WIDTH - 1:0] countIn,
	output [COUNT_WIDTH - 1:0] countOut,
	 
	// Delay
	input en_n
	
);

/**********
 * Internal Signals
**********/
function integer log2; //This is a macro function (no hardware created) which finds the log2, returns log2
   input [31:0] val; //input to the function
   integer 	i;
   begin
      log2 = 0;
      for(i = 0; 2**i < val; i = i + 1)
		log2 = i + 1;
   end
endfunction

reg [COUNT_WIDTH - 1:0] countValue;	// Register to hold the count value
wire [COUNT_WIDTH - 1:0] countMax = MAX_COUNT;
/**********
 * Glue Logic 
 **********/
/**********
 * Synchronous Logic
 **********/
 always @(posedge clk) begin
	if(rst) begin
		countValue <= {(COUNT_WIDTH){1'b0}};		// Reset the count to zero
	end else begin
		if(load) begin
			countValue <= countIn;					// If load = 1 then set count value to the input value
		end else if(pause) begin
			countValue <= countValue;				// If pause = 1 hold the value
		end else if(countValue^countMax) begin
			countValue <= countValue + 1;			// Increase count value by 1
		end else begin
			countValue <= {(COUNT_WIDTH){1'b0}};	// If at MAX_COUNT next value is 0
		end
	end
 end
 
/**********
 * Glue Logic 
 **********/
/**********
 * Components
 **********/
  delay #(
	.BIT_WIDTH(COUNT_WIDTH),
	.DEPTH(1),
	.DELAY(DELAY)
 ) U_IP(
	.clk(clk),
	.rst(rst),
	.en_n(en_n),
	.dataIn(countValue),
	.dataOut(countOut)
 );
 
/**********
 * Output Combinatorial Logic
 **********/
 //assign countOut = countValue;		// Assign output
 
endmodule



/*
DESCRIPTION:
Forwarding Unit

NOTES:
TODO:
*/

module forwardingUnit_r0 #(
	parameter BIT_WIDTH = 5
)(
	input [BIT_WIDTH - 1:0] ID_EX_Rs,
	input [BIT_WIDTH - 1:0] ID_EX_Rt,
	input [BIT_WIDTH - 1:0] EX_MEM_Rd,
	input [BIT_WIDTH - 1:0] MEM_WB_Rd,
	input EX_MEM_RegWrite,
	input MEM_WB_RegWrite,
	output [1:0] ForwardA,
	output [1:0] ForwardB
);

/**********
 * Internal Signals
 **********/
 reg [1:0] ForwardA_tmp;
 reg [1:0] ForwardB_tmp;
	 
/**********
 * Glue Logic 
 **********/
/**********
 * Synchronous Logic
 **********/
/**********
 * Glue Logic 
 **********/
/**********
 * Components
 **********/
/**********
 * Output Combinatorial Logic
 **********/
	always @(ID_EX_Rs, ID_EX_Rt, EX_MEM_Rd, MEM_WB_Rd, EX_MEM_RegWrite, MEM_WB_RegWrite) begin
		ForwardA_tmp = 2'b00;
		ForwardB_tmp = 2'b00;
	
		if((EX_MEM_RegWrite == 1'b1) && (EX_MEM_Rd != 5'b00000) && (EX_MEM_Rd == ID_EX_Rs)) begin
			ForwardA_tmp = 2'b10;
		end else if((MEM_WB_RegWrite == 1'b1) && (MEM_WB_Rd != 5'b00000) && (MEM_WB_Rd == ID_EX_Rs)) begin
			ForwardA_tmp = 2'b01;
		end
		
		if((EX_MEM_RegWrite == 1'b1) && (EX_MEM_Rd != 5'b00000) && (EX_MEM_Rd == ID_EX_Rt)) begin
			ForwardB_tmp = 2'b10;
		end else if((MEM_WB_RegWrite == 1'b1) && (MEM_WB_Rd != 5'b00000) && (MEM_WB_Rd == ID_EX_Rt)) begin
			ForwardB_tmp = 2'b01;
		end
	end

	assign ForwardA = ForwardA_tmp;
	assign ForwardB = ForwardB_tmp;
endmodule




/*
DESCRIPTION:
Hazard Detection Unit

NOTES:
TODO:
*/

module hazardDetectionUnit_r0 (
	input [5:0] IF_ID_Opcode,
	input [5:0] IF_ID_Funcode,
	input [4:0] IF_ID_Rs,
	input [4:0] IF_ID_Rt,
	input ID_EX_MemRead,
	input [4:0] ID_EX_Rt,
	input equal,
	input [4:0] ID_EX_Rd,
	input [4:0] EX_MEM_Rd,
	input ID_EX_RegWrite,
	input EX_MEM_RegWrite,
	output reg PCWrite,
	output reg ID_EX_CtrlFlush,
	output reg IF_ID_Flush,
	output reg IF_ID_Hold
);

/**********
 * Internal Signals
 **********/
 localparam j		= 6'h02;
 localparam jal		= 6'h03;
 localparam beq		= 6'h04;
 localparam bne		= 6'h05;
 
 localparam R_Type 	= 6'h00;
 localparam fun_jr		= 6'h08;
 
/**********
 * Glue Logic 
 **********/
/**********
 * Synchronous Logic
 **********/
/**********
 * Glue Logic 
 **********/
/**********
 * Components
 **********/
/**********
 * Output Combinatorial Logic
 **********/
	always @(IF_ID_Opcode, IF_ID_Rs, IF_ID_Rt, ID_EX_MemRead, ID_EX_Rt, equal, ID_EX_Rd, EX_MEM_Rd, ID_EX_RegWrite, EX_MEM_RegWrite) begin
		if((ID_EX_MemRead == 1'b1) && ((ID_EX_Rt == IF_ID_Rs) || (ID_EX_Rt == IF_ID_Rt))) begin
			PCWrite <= 1'b0;
			ID_EX_CtrlFlush <= 1'b1;
			IF_ID_Flush <= 1'b0;
			IF_ID_Hold <= 1'b1;
		end else if(((IF_ID_Opcode == beq) || (IF_ID_Opcode == bne) || ((IF_ID_Opcode == R_Type) && (IF_ID_Funcode == fun_jr))) 
		&& (((ID_EX_Rd == IF_ID_Rs || ID_EX_Rd == IF_ID_Rt) && (ID_EX_RegWrite == 1'b1)) || ((EX_MEM_Rd == IF_ID_Rs || EX_MEM_Rd == IF_ID_Rt) && (EX_MEM_RegWrite == 1'b1)))) begin
			PCWrite <= 1'b0;
			ID_EX_CtrlFlush <= 1'b1;
			IF_ID_Flush <= 1'b0;
			IF_ID_Hold <= 1'b1;
		end else if((IF_ID_Opcode == j) /*|| (IF_ID_Opcode == jal)*/ || ((IF_ID_Opcode == R_Type) && (IF_ID_Funcode == fun_jr)) || (IF_ID_Opcode == beq && (equal == 1'b1)) || (IF_ID_Opcode == bne && (equal == 1'b0))) begin
			PCWrite <= 1'b1;
			ID_EX_CtrlFlush <= 1'b0;
			IF_ID_Flush <= 1'b1;
			IF_ID_Hold <= 1'b0;
		end else if(IF_ID_Opcode == jal) begin
			PCWrite <= 1'b1;
			ID_EX_CtrlFlush <= 1'b0;
			IF_ID_Flush <= 1'b1;
			IF_ID_Hold <= 1'b0;
		end else begin
			PCWrite <= 1'b1;
			ID_EX_CtrlFlush <= 1'b0;
			IF_ID_Flush <= 1'b0;
			IF_ID_Hold <= 1'b0;
		end
	end	
endmodule



/*
DESCRIPTION
Controller for Memory

NOTES
	
TODO

*/

module memcontroller_r0 (
	input [5:0] opcode,
	input [1:0] MemSelect,
	output [3:0] MemWrite,
	output [3:0] MemRead,
	output MemMux1Sel,
	output [1:0] MemMux2Sel,
	output [1:0] MemMux3Sel
);

/**********
 * Internal Signals
**********/
reg [3:0] MemWrite_tmp;
reg [3:0] MemRead_tmp;
reg MemMux1Sel_tmp;
reg [1:0] MemMux2Sel_tmp;
reg [1:0] MemMux3Sel_tmp;

/**********
 * Glue Logic 
 **********/
 localparam lw		= 6'h23;
 localparam lbu		= 6'h24;
 localparam lhu		= 6'h25;
 localparam sb		= 6'h28;
 localparam sh		= 6'h29;
 localparam sw 		= 6'h2B;
 
/**********
 * Synchronous Logic
 **********/
/**********
 * Glue Logic 
 **********/
/**********`
 * Components
 **********/
/**********
 * Output Combinatorial Logic
 **********/
 always @(opcode, MemSelect) begin
	MemWrite_tmp <= 4'b0000;
	MemRead_tmp <= 4'b0000;
	MemMux1Sel_tmp <= 1'b0;
	MemMux2Sel_tmp <= 2'b00;
	MemMux3Sel_tmp <= 2'b00;
 
	casez({opcode, MemSelect})
	 // Store Byte
	 {sb, 2'b00} : begin
		MemWrite_tmp <= 4'b0001;
	 end
	 {sb, 2'b01} : begin
		MemWrite_tmp <= 4'b0010;
		MemMux1Sel_tmp <= 1'b0;
	 end
	 {sb, 2'b10} : begin
		MemWrite_tmp <= 4'b0100;
		MemMux2Sel_tmp <= 2'b00;
	 end
	 {sb, 2'b11} : begin
		MemWrite_tmp <= 4'b1000;
		MemMux3Sel_tmp <= 2'b00;
	 end
	 
	 // Store Half Word
	 {sh, 2'b00} : begin
		MemWrite_tmp <= 4'b0011;
		MemMux1Sel_tmp <= 1'b1;
	 end
	 {sh, 2'b01} : begin
		MemWrite_tmp <= 4'b0110;
		MemMux1Sel_tmp <= 1'b0;
		MemMux2Sel_tmp <= 2'b01;
	 end
	 {sh, 2'b10} : begin
		MemWrite_tmp <= 4'b1100;
		MemMux2Sel_tmp <= 2'b00;
		MemMux3Sel_tmp <= 2'b01;
	 end

	 // Store Word
	 {sw, 2'b??} : begin
		MemWrite_tmp <= 4'b1111;
		MemMux1Sel_tmp <= 1'b1;
		MemMux2Sel_tmp <= 2'b10;
		MemMux3Sel_tmp <= 2'b10;
	 end
	 
	 // Load Byte
	 {lbu, 2'b00} : begin
		MemRead_tmp <= 4'b0001;
	 end
	 {lbu, 2'b01} : begin
		MemRead_tmp <= 4'b0010;
	 end
	 {lbu, 2'b10} : begin
		MemRead_tmp <= 4'b0100;
	 end
	 {lbu, 2'b11} : begin
		MemRead_tmp <= 4'b1000;
	 end
	 
	 // Load Half Word
	 {lhu, 2'b00} : begin
		MemRead_tmp <= 4'b0011;
	 end
	 {lhu, 2'b01} : begin
		MemRead_tmp <= 4'b0110;
	 end
	 {lhu, 2'b10} : begin
		MemRead_tmp <= 4'b1100;
	 end

	 // Load Word
	 {lw, 2'b??} : begin
		MemRead_tmp <= 4'b1111;
	 end
	endcase
 end
 
 assign MemWrite = MemWrite_tmp;
 assign MemRead = MemRead_tmp;
 assign MemMux1Sel = MemMux1Sel_tmp;
 assign MemMux2Sel = MemMux2Sel_tmp;
 assign MemMux3Sel = MemMux3Sel_tmp;
endmodule



/*
DESCRIPTION
Module for the logic with load instructions and the memory

NOTES

TODO

*/

module memout_r0 #(
	parameter DATA_WIDTH = 8
)(
	input [DATA_WIDTH - 1:0] Mem0Out,
	input [DATA_WIDTH - 1:0] Mem1Out,
	input [DATA_WIDTH - 1:0] Mem2Out,
	input [DATA_WIDTH - 1:0] Mem3Out,
	input [1:0] MemSel,
	input [5:0] Opcode, 
	output [31:0] MemOut
);
/**********
 *  Array Packing Defines 
 **********/
/**********
 * Internal Signals
**********/
reg [31 - 1:0] MemOut_tmp;

/**********
 * Glue Logic 
 **********/
 localparam lw		= 6'h23;
 localparam lbu		= 6'h24;
 localparam lhu		= 6'h25;
 
/**********
 * Synchronous Logic
 **********/
/**********
 * Glue Logic 
 **********/
/**********
 * Components
 **********/
/**********
 * Output Combinatorial Logic
 **********/
 always @(Opcode, MemSel, Mem0Out, Mem1Out, Mem2Out, Mem3Out) begin
	if(Opcode == lbu) begin
		if(MemSel == 2'b00) begin
			MemOut_tmp <= {24'h000000, Mem0Out};
		end else if(MemSel == 2'b01) begin
			MemOut_tmp <= {24'h000000, Mem1Out};
		end else if(MemSel == 2'b10) begin
			MemOut_tmp <= {24'h000000, Mem2Out};
		end else if(MemSel == 2'b11) begin
			MemOut_tmp <= {24'h000000, Mem3Out};
		end
	end else if(Opcode == lhu) begin
		if(MemSel == 2'b00) begin
			MemOut_tmp <= {16'b0000, Mem1Out, Mem0Out};
		end else if(MemSel == 2'b01) begin
			MemOut_tmp <= {16'b0000, Mem2Out, Mem1Out};
		end else begin
			MemOut_tmp <= {16'b0000, Mem3Out, Mem2Out};
		end
	end else if(Opcode == lw) begin
		MemOut_tmp <= {Mem3Out, Mem2Out, Mem1Out, Mem0Out};
	end
 end 
 
 assign MemOut = MemOut_tmp;
endmodule



/*
DESCRIPTION
Mult Module

NOTES

TODO

*/

module mult_r0 #(
	parameter DATA_WIDTH = 32
)(
	input [DATA_WIDTH - 1:0] input1,
	input [DATA_WIDTH - 1:0] input2,
	output[2*DATA_WIDTH - 1:0] dataOut,
	output C,
	output Z,
	output V,
	output S
);
/**********
 *  Array Packing Defines 
 **********/
/**********
 * Internal Signals
**********/
reg [2*DATA_WIDTH - 1:0] tmpMult;
reg Ctmp, Ztmp, Vtmp, Stmp;

/**********
 * Glue Logic 
 **********/
/**********
 * Synchronous Logic
 **********/
/**********
 * Glue Logic 
 **********/
/**********
 * Components
 **********/
/**********
 * Output Combinatorial Logic
 **********/
 always @(input1, input2) begin
	Ctmp = 0;
	Ztmp = 0;
	Vtmp = 0;
	Stmp = 0;
	
	tmpMult = input1 * input2;
	
	if(tmpMult[DATA_WIDTH-1:0] == {(2*DATA_WIDTH){1'b0}}) begin
		Ztmp = 1;
	end

	Stmp = tmpMult[2*DATA_WIDTH - 1];
 end
 
 assign dataOut = tmpMult;
 assign C = Ctmp;
 assign Z = Ztmp;
 assign V = Vtmp;
 assign S = Stmp;
 
endmodule



/*
DESCRIPTION
Mux Wrapper

NOTES
	BIT_WIDTH 	// number of bits for input(s) and output
	DEPTH 		// number of inputs to the mux
	SEL_WIDTH	// width of the select for the mux
	
	dataIn		// vectorized inputs to the mux
	sel			// select value
	dataOut		// output from the selected input
	
TODO

*/

module mux #(
	parameter BIT_WIDTH = 4,
	parameter DEPTH = 2,
	parameter SEL_WIDTH = log2(DEPTH)
)(
	input [BIT_WIDTH*DEPTH - 1:0] dataIn,
	input [SEL_WIDTH - 1:0] sel,
	output [BIT_WIDTH - 1:0] dataOut
);

/**********
 * Internal Signals
**********/
function integer log2; //This is a macro function (no hardware created) which finds the log2, returns log2
   input [31:0] val; //input to the function
   integer 	i;
   begin
      log2 = 0;
      for(i = 0; 2**i < val; i = i + 1)
		log2 = i + 1;
   end
endfunction 

/**********
 * Glue Logic 
 **********/
/**********
 * Synchronous Logic
 **********/

/**********
 * Glue Logic 
 **********/
/**********
 * Components
 **********/
mux_r0 #(
	.BIT_WIDTH(BIT_WIDTH),
	.DEPTH(DEPTH),
	.SEL_WIDTH(SEL_WIDTH)
)U_IP(
	.dataIn(dataIn),
	.sel(sel),
	.dataOut(dataOut)
);
/**********
 * Output Combinatorial Logic
 **********/
endmodule



/*
DESCRIPTION
Generic Mux

NOTES
	BIT_WIDTH 	// number of bits for input(s) and output
	DEPTH 		// number of inputs to the mux
	SEL_WIDTH	// width of the select for the mux
	
	dataIn		// vectorized inputs to the mux
	sel			// select value
	dataOut		// output from the selected input
	
TODO
*/

module mux_r0 #(
	parameter BIT_WIDTH = 4,
	parameter DEPTH = 2,
	parameter SEL_WIDTH = log2(DEPTH)
)(
	input [BIT_WIDTH*DEPTH - 1:0] dataIn,
	input [SEL_WIDTH - 1:0] sel,
	output [BIT_WIDTH - 1:0] dataOut
);

/**********
 *  Array Packing Defines 
 **********/
`define PACK_ARRAY(PK_WIDTH,PK_DEPTH,PK_SRC,PK_DEST, BLOCK_ID, GEN_VAR)    genvar GEN_VAR; generate for (GEN_VAR=0; GEN_VAR<(PK_DEPTH); GEN_VAR=GEN_VAR+1) begin: BLOCK_ID assign PK_DEST[((PK_WIDTH)*GEN_VAR+((PK_WIDTH)-1)):((PK_WIDTH)*GEN_VAR)] = PK_SRC[GEN_VAR][((PK_WIDTH)-1):0]; end endgenerate
`define UNPACK_ARRAY(PK_WIDTH,PK_DEPTH,PK_DEST,PK_SRC, BLOCK_ID, GEN_VAR)  genvar GEN_VAR; generate for (GEN_VAR=0; GEN_VAR<(PK_DEPTH); GEN_VAR=GEN_VAR+1) begin: BLOCK_ID assign PK_DEST[GEN_VAR][((PK_WIDTH)-1):0] = PK_SRC[((PK_WIDTH)*GEN_VAR+(PK_WIDTH-1)):((PK_WIDTH)*GEN_VAR)]; end endgenerate

/**********
 * Internal Signals
 **********/
function integer log2; //This is a macro function (no hardware created) which finds the log2, returns log2
   input [31:0] val; //input to the function
   integer 	i;
   begin
      log2 = 0;
      for(i = 0; 2**i < val; i = i + 1)
		log2 = i + 1;
   end
endfunction 

wire [BIT_WIDTH - 1:0] tmp [DEPTH - 1:0]; //input as array

/**********
 * Glue Logic 
 **********/
`UNPACK_ARRAY(BIT_WIDTH,DEPTH,tmp,dataIn, U_BLK_0, idx_0)
 
/**********
 * Synchronous Logic
 **********/
/**********
 * Glue Logic 
 **********/
/**********
 * Components
 **********/
/**********
 * Output Combinatorial Logic
 **********/
 assign dataOut = tmp[sel];
endmodule



// Data Memory

module ram(q, a, d, rst, we, re, clk);
   output[7:0] q;
   input [7:0] d;
   input [5:0] a;
   input rst, we, re, clk;
   
   reg [7:0] qtmp;
   
   reg [7:0] mem [63:0];
   integer i;
   
   always @(posedge clk) begin
        if(rst == 1'b1) begin
			for(i = 0; i < 64; i=i+1) begin
				mem[i] <= {(8){1'b0}};
			end
		end else if (we == 1'b1) begin
            mem[a] <= d;
		end
   end
   
   always @* begin
		if(re == 1'b1) begin
			qtmp <= mem[a];
		end else begin
			qtmp <= {(8){1'b0}};
		end
   end
   
   assign q = qtmp;
endmodule



/*
DESCRIPTION
Generic Register File

NOTES
	DATA_WIDTH	// word length
	RD_DEPTH	// number of parallel reads
	REG_DEPTH	// depth of the register file
	ADDR_WIDTH	// address width, operand width
	rst			// synchronous reset
	wr			// write enable
	rr			// rg read address as vecotrized array
	rw			// reg write address
	d			// input datat to be written to reg
	q			// output data from reg reads as vectorized array
	
TODO
*/

module registerFile #(
	parameter DATA_WIDTH = 32,
	parameter RD_DEPTH = 2,
	parameter REG_DEPTH = 32,
	parameter ADDR_WIDTH = log2(REG_DEPTH)
)(
	input clk,
	input rst,
	input wr,
	input [ADDR_WIDTH*RD_DEPTH - 1:0] rr,
	input [ADDR_WIDTH - 1:0] rw,
	input [DATA_WIDTH - 1:0] d,
	output [DATA_WIDTH*RD_DEPTH - 1:0] q
);

/**********
 * Internal Signals
**********/
function integer log2; //This is a macro function (no hardware created) which finds the log2, returns log2
   input [31:0] val; //input to the function
   integer 	i;
   begin
      log2 = 0;
      for(i = 0; 2**i < val; i = i + 1)
		log2 = i + 1;
   end
endfunction 

/**********
 * Glue Logic 
 **********/
/**********
 * Synchronous Logic
 **********/

/**********
 * Glue Logic 
 **********/
/**********
 * Components
 **********/
registerFile_r0 #(
	.DATA_WIDTH(DATA_WIDTH),
	.RD_DEPTH(RD_DEPTH),
	.REG_DEPTH(REG_DEPTH),
	.ADDR_WIDTH(ADDR_WIDTH)
)U_IP(
	.clk(clk),
	.rst(rst),
	.wr(wr),
	.rr(rr),
	.rw(rw),
	.d(d),
	.q(q)
);
/**********
 * Output Combinatorial Logic
 **********/
endmodule


/*
DESCRIPTION
Generic Register File

NOTES
	DATA_WIDTH	// word length
	RD_DEPTH	// number of parallel reads
	REG_DEPTH	// depth of the register file
	ADDR_WIDTH	// address width, operand width
	rst			// synchronous reset
	wr			// write enable
	rr			// rg read address as vecotrized array
	rw			// reg write address
	d			// input datat to be written to reg
	q			// output data from reg reads as vectorized array
	
TODO
*/

module registerFile_r0 #(
	parameter DATA_WIDTH = 32,
	parameter RD_DEPTH = 2,
	parameter REG_DEPTH = 32,
	parameter ADDR_WIDTH = log2(REG_DEPTH)
)(
	input clk,
	input rst,
	input wr,
	input [ADDR_WIDTH*RD_DEPTH - 1:0] rr,
	input [ADDR_WIDTH - 1:0] rw,
	input [DATA_WIDTH - 1:0] d,
	output [DATA_WIDTH*RD_DEPTH - 1:0] q
);

/**********
 *  Array Packing Defines 
 **********/
`define PACK_ARRAY(PK_WIDTH,PK_DEPTH,PK_SRC,PK_DEST, BLOCK_ID, GEN_VAR)    genvar GEN_VAR; generate for (GEN_VAR=0; GEN_VAR<(PK_DEPTH); GEN_VAR=GEN_VAR+1) begin: BLOCK_ID assign PK_DEST[((PK_WIDTH)*GEN_VAR+((PK_WIDTH)-1)):((PK_WIDTH)*GEN_VAR)] = PK_SRC[GEN_VAR][((PK_WIDTH)-1):0]; end endgenerate
`define UNPACK_ARRAY(PK_WIDTH,PK_DEPTH,PK_DEST,PK_SRC, BLOCK_ID, GEN_VAR)  genvar GEN_VAR; generate for (GEN_VAR=0; GEN_VAR<(PK_DEPTH); GEN_VAR=GEN_VAR+1) begin: BLOCK_ID assign PK_DEST[GEN_VAR][((PK_WIDTH)-1):0] = PK_SRC[((PK_WIDTH)*GEN_VAR+(PK_WIDTH-1)):((PK_WIDTH)*GEN_VAR)]; end endgenerate

/**********
 * Internal Signals
**********/
function integer log2; //This is a macro function (no hardware created) which finds the log2, returns log2
   input [31:0] val; //input to the function
   integer 	i;
   begin
      log2 = 0;
      for(i = 0; 2**i < val; i = i + 1)
		log2 = i + 1;
   end
endfunction

reg [DATA_WIDTH - 1:0] registers [REG_DEPTH - 1:0];	// registers in the register file
wire [DATA_WIDTH - 1:0] readouttmp [RD_DEPTH - 1:0];	// tmp for the read values from the registers

integer i,j;

wire [ADDR_WIDTH - 1:0] rrtmp [RD_DEPTH - 1:0];		// array for the address values
wire [DATA_WIDTH*RD_DEPTH - 1:0] qtmp;				// vector for the output values
wire [DATA_WIDTH*REG_DEPTH - 1:0] regvector;

/**********
 * Glue Logic 
 **********/
 `UNPACK_ARRAY(ADDR_WIDTH, RD_DEPTH, rrtmp, rr, U_BLK_0, idx_0)
 
 //assign regswires = registers;
/**********
 * Synchronous Logic
 **********/
 always @(negedge clk /*posedge clk*/) begin
	if(rst) begin
		for(i=0; i<REG_DEPTH; i=i+1) begin
			registers[i] <= {(DATA_WIDTH){1'b0}};	// Reset each register to 0
		end
	end else begin		
		if(wr == 1'b1) begin
			registers[rw] <= d;	// Write to the register if write is enabled
			 
		end
		
		registers[0] <= {(DATA_WIDTH){1'b0}};
	end
	// $display( "wr :%b\twrite data :%h\t data in reg : %h\t rw : %h" , wr, d , registers[0] , rw  ) ;
	$display( "in registerFile wr :%b\twrite data :%h\t data in reg : %h\t rw : %h" , wr, d , registers[0] , 0  ) ;
	$display( "in registerFile wr :%b\twrite data :%h\t data in reg : %h\t rw : %h" , wr, d , registers[1] , 1  ) ;
	$display( "in registerFile wr :%b\twrite data :%h\t data in reg : %h\t rw : %h" , wr, d , registers[31] , 31 ) ;
	// $display( "in registerFile wr :%b\twrite data :%h\t data in reg : %h\t rw : %h" , wr, d , registers[6] , 6  ) ;

 end
 
/**********
 * Glue Logic 
 **********/
 `PACK_ARRAY(DATA_WIDTH, RD_DEPTH, readouttmp, qtmp, U_BLK_1, idx_1)	// Turn array into vector for output
 `PACK_ARRAY(DATA_WIDTH, REG_DEPTH, registers, regvector, U_BLK_2, idx_2)	// Turn array into vector for output
 
/**********
 * Components
 **********/
 genvar k;
 generate 
	for(k=0; k<RD_DEPTH; k=k+1) begin : regmuxes
		mux #(
			.BIT_WIDTH(DATA_WIDTH),
			.DEPTH(REG_DEPTH),
			.SEL_WIDTH(ADDR_WIDTH)
		)U_IP(
			.dataIn(regvector),
			.sel(rrtmp[k]),
			.dataOut(readouttmp[k])
		);
	end
 endgenerate
 
/**********
 * Output Combinatorial Logic
 **********/
 assign q = qtmp;
 
endmodule


// Instruction Memory
// 128 Words Long
// $readmemh will load the given program from the .hex file

module rom(q, a);
   integer i =0 ; 
   output[31:0] q;
   input [6:0] a;
  
   reg [31:0] mem [127:0];
	
   initial begin
    $readmemh("data.hex", mem, 0, 127);
    // $monitor("instruction fetched %h", q );
  end
	
   assign q = mem[a];
	
endmodule



/*
DESCRIPTION
Sign Extender

NOTES
	IN_WIDTH 	// Width of the input
	OUT_WIDTH 	// Width of the output
	
	isSigned	// Whether the input is signed or unsigned
	dataIn		// Input data
	dataOut		// Output data
TODO

*/

 module signextender_r0 #(
 	parameter IN_WIDTH = 16,
 	parameter OUT_WIDTH = 32,
 	parameter DEPTH = 1,
 	parameter DELAY = 0
 )(
 	input [DEPTH*IN_WIDTH - 1:0] dataIn,
 	output [DEPTH*OUT_WIDTH - 1:0] dataOut,
 	input isSigned,
	
 	// Delay Inputs
 	input clk,
 	input rst,
 	input en_n
 );

 /**********
  *  Array Packing Defines 
  **********/
 `define PACK_ARRAY(PK_WIDTH,PK_DEPTH,PK_SRC,PK_DEST, BLOCK_ID, GEN_VAR)    genvar GEN_VAR; generate for (GEN_VAR=0; GEN_VAR<(PK_DEPTH); GEN_VAR=GEN_VAR+1) begin: BLOCK_ID assign PK_DEST[((PK_WIDTH)*GEN_VAR+((PK_WIDTH)-1)):((PK_WIDTH)*GEN_VAR)] = PK_SRC[GEN_VAR][((PK_WIDTH)-1):0]; end endgenerate
 `define UNPACK_ARRAY(PK_WIDTH,PK_DEPTH,PK_DEST,PK_SRC, BLOCK_ID, GEN_VAR)  genvar GEN_VAR; generate for (GEN_VAR=0; GEN_VAR<(PK_DEPTH); GEN_VAR=GEN_VAR+1) begin: BLOCK_ID assign PK_DEST[GEN_VAR][((PK_WIDTH)-1):0] = PK_SRC[((PK_WIDTH)*GEN_VAR+(PK_WIDTH-1)):((PK_WIDTH)*GEN_VAR)]; end endgenerate

 /**********
  * Internal Signals
 **********/
 wire [IN_WIDTH - 1:0] tmpIn [DEPTH - 1:0];
 reg [OUT_WIDTH - 1:0] extTmp [DEPTH - 1:0];	// temp to hold the vector being created
 wire [DEPTH*OUT_WIDTH - 1:0] outTmp;

 integer i;
 /**********
  * Glue Logic 
  **********/
   `UNPACK_ARRAY(IN_WIDTH, DEPTH, tmpIn, dataIn, U_BLK_0, idx_0)
  
 /**********
  * Synchronous Logic
  **********/
 /**********
  * Glue Logic 
  **********/
  `PACK_ARRAY(OUT_WIDTH, DEPTH, extTmp, outTmp, U_BLK_1, idx_1)
 
 /**********
  * Components
  **********/
  delay #(
 	.BIT_WIDTH(OUT_WIDTH),
 	.DEPTH(DEPTH),
 	.DELAY(DELAY)
  ) U_IP(
 	.clk(clk),
 	.rst(rst),
 	.en_n(en_n),
 	.dataIn(outTmp),
 	.dataOut(dataOut)
  );
 
 /**********
  * Output Combinatorial Logic
  **********/
 
  always @( tmpIn[0] ,isSigned) begin			/// tmpIn ,
 	for(i=0; i<DEPTH; i=i+1) begin
 		if(isSigned == 1'b1) begin
 			extTmp[i][OUT_WIDTH - 1:IN_WIDTH] <= {(OUT_WIDTH - IN_WIDTH){tmpIn[i][IN_WIDTH - 1]}};
 			extTmp[i][IN_WIDTH - 1:0] <= tmpIn[i];
 		end else begin
 			extTmp[i][OUT_WIDTH - 1:IN_WIDTH] <= {(OUT_WIDTH - IN_WIDTH){1'b0}};
 			extTmp[i][IN_WIDTH - 1:0] <= tmpIn[i];
 		end
 	end
  end
 
  //assign dataOut = outTmp;	// assign the output to the newly created vector
 endmodule



/*
DESCRIPTION
Sub Module

NOTES

TODO

*/

module sub_r0 #(
	parameter DATA_WIDTH = 32
)(
	input [DATA_WIDTH - 1:0] input1,
	input [DATA_WIDTH - 1:0] input2,
	output[DATA_WIDTH - 1:0] dataOut,
	output C,
	output Z,
	output V,
	output S
);
/**********
 *  Array Packing Defines 
 **********/
/**********
 * Internal Signals
**********/
reg [DATA_WIDTH:0] tmpSub;
reg Ctmp, Ztmp, Vtmp, Stmp;

/**********
 * Glue Logic 
 **********/
/**********
 * Synchronous Logic
 **********/
/**********
 * Glue Logic 
 **********/
/**********
 * Components
 **********/
/**********
 * Output Combinatorial Logic
 **********/
 always @(input1, input2) begin
	Ctmp = 0;
	Ztmp = 0;
	Vtmp = 0;
	Stmp = 0;
	
	tmpSub = input1 - input2;
	
	Ctmp = tmpSub[DATA_WIDTH];
	
	if(tmpSub[DATA_WIDTH-1:0] == {(DATA_WIDTH){1'b0}}) begin
		Ztmp = 1;
	end
	
	if((input1[DATA_WIDTH - 1] != input2[DATA_WIDTH - 1]) && (tmpSub[DATA_WIDTH - 1] == input2[DATA_WIDTH - 1])) begin
		Vtmp = 1;
	end

	Stmp = tmpSub[DATA_WIDTH - 1];
 end
 
 assign dataOut = tmpSub[DATA_WIDTH - 1:0];
 assign C = Ctmp;
 assign Z = Ztmp;
 assign V = Vtmp;
 assign S = Stmp;
 
endmodule


/*
DESCRIPTION
Main Controller

NOTES

TODO
*/

module controller_r0 (
	input [5:0] opcode,
	input [5:0] funcode, 
	output [1:0] RegDst,				// 0 = 20:16, 1 = 15:11, 2 = $31
	output ALUSrc,
	output MemtoReg,
	output RegWrite,
	output [1:0] RegWriteSrc,	// 0 = output from memory, 1 = 16-bit left-shifted value for lui, 2 = PC + 4 for JAL
	output MemRead,
	//output MemRead2,
	//output MemRead3,
	output MemWrite ,
	//output MemWrite1,
	//output MemWrite2,
	//output MemWrite3,
	output Jump,
	output JumpRegID,
	output BranchBEQ,
	output BranchBNE,
	output [2:0] ALUOp,
	output isSigned
);

/**********
 * Internal Signals
**********/
 reg [1:0] RegDst_tmp;
 reg ALUSrc_tmp;
 reg MemtoReg_tmp;
 reg RegWrite_tmp;
 reg [1:0] RegWriteSrc_tmp;
 reg MemRead_tmp;
 //reg MemRead1_tmp;
 //reg MemRead2_tmp;
 //reg MemRead3_tmp;
 reg MemWrite_tmp;
 //reg MemWrite1_tmp;
 //reg MemWrite2_tmp;
 //reg MemWrite3_tmp;
 reg BranchBEQ_tmp;
 reg BranchBNE_tmp;
 reg Jump_tmp;
 reg JumpRegID_tmp;
 reg [2:0] ALUOp_tmp;
 reg isSigned_tmp;
	
/**********
 * Glue Logic 
 **********/
 localparam R_Type 	= 6'h00;
 localparam j		= 6'h02;
 localparam jal		= 6'h03;
 localparam beq		= 6'h04;
 localparam bne		= 6'h05;
 //localparam blez	= 6'h06;
 //localparam bgtz	= 6'h07;
 localparam addi 	= 6'h08;
 localparam addiu 	= 6'h09;
 localparam slti 	= 6'h0A;
 localparam sltiu	= 6'h0B;
 localparam andi	= 6'h0C;
 localparam ori		= 6'h0D;
 localparam xori	= 6'h0E;
 localparam lui		= 6'h0F;
 //localparam lb	= 6'h20;
 //localparam lh	= 6'h21;
 //localparam lwl	= 6'h22;
 localparam lw		= 6'h23;
 localparam lbu		= 6'h24;
 localparam lhu		= 6'h25;
 //localparam lwr	= 6'h26;
 localparam sb		= 6'h28;
 localparam sh		= 6'h29;
 //localparam swl	= 6'h2A;
 localparam sw 		= 6'h2B;
 //localparam swr	= 6'h2E;
 
 localparam ALUadd = 3'b000;	// for addi, addiu, lw, lbu, lhu, sb, sh, sw
 localparam ALUsub = 3'b001;	// for beq, bne
 localparam ALUand = 3'b010;	// for andi
 localparam ALUor  = 3'b011;	// for ori
 localparam ALUxor = 3'b100;	// for xori
 localparam ALUslt = 3'b101;	// for slti, sltiu
 localparam ALURtp = 3'b111;	// for all R-Type
 
 localparam fun_jr		= 6'h08;
 
/**********
 * Synchronous Logic
 **********/
/**********
 * Glue Logic 
 **********/
/**********
 * Components
 **********/
/**********
 * Output Combinatorial Logic
 **********/
 /*always @(funcode) begin
	JumpRegID_tmp = 0;
	
	if(funcode == fun_jr) begin
		JumpRegID_tmp = 1;
	end
 end*/
 
 always @(opcode, funcode) begin
	RegDst_tmp = 2'b00;
	ALUSrc_tmp = 0;
	MemtoReg_tmp = 0;
	RegWrite_tmp = 0;
	RegWriteSrc_tmp = 2'b00;
	MemRead_tmp = 0;
	//MemRead1_tmp = 0;
	//MemRead2_tmp = 0;
	//MemRead3_tmp = 0;
	//MemWrite0_tmp = 0;
	//MemWrite1_tmp = 0;
	//MemWrite2_tmp = 0;
	//MemWrite3_tmp = 0;
	BranchBEQ_tmp = 0;
	BranchBNE_tmp = 0;
	Jump_tmp = 0;
	ALUOp_tmp = 3'b000;
	isSigned_tmp = 0;
	
	JumpRegID_tmp = 0;
	
	case(opcode)
		R_Type: begin
			RegDst_tmp = 2'b01;
			RegWrite_tmp = 1;
			ALUOp_tmp = ALURtp;
			
			if(funcode == fun_jr) begin
				JumpRegID_tmp = 1;
			end
		end
		
		addi: begin
			ALUSrc_tmp = 1;
			RegWrite_tmp = 1;
			ALUOp_tmp = ALUadd;
			isSigned_tmp = 1;
		end
		
		addiu: begin
			ALUSrc_tmp = 1;
			RegWrite_tmp = 1;
			ALUOp_tmp = ALUadd;
		end
		
		slti: begin
			ALUSrc_tmp = 1;
			ALUOp_tmp = ALUslt;
			RegWrite_tmp = 1;
			isSigned_tmp = 1;
		end
		
		sltiu: begin
			ALUSrc_tmp = 1;
			ALUOp_tmp = ALUslt;
			RegWrite_tmp = 1;
		end
		
		andi: begin
			ALUSrc_tmp = 1;
			RegWrite_tmp = 1;
			ALUOp_tmp = ALUand;
			//isSigned_tmp = 1;
		end
		
		ori: begin
			ALUSrc_tmp = 1;
			RegWrite_tmp = 1;
			ALUOp_tmp = ALUor;
			//isSigned_tmp = 1;
		end
		
		xori: begin
			ALUSrc_tmp = 1;
			RegWrite_tmp = 1;
			ALUOp_tmp = ALUxor;
			//isSigned_tmp = 1;
		end
		
		lui: begin
			RegWrite_tmp = 1;
			RegWriteSrc_tmp = 2'b01;
			isSigned_tmp = 1;
		end
		
		lbu: begin
			ALUSrc_tmp = 1;
			MemtoReg_tmp = 1;
			RegWrite_tmp = 1;
			MemRead_tmp = 1;
			ALUOp_tmp = ALUadd;
		end
		
		lhu: begin
			ALUSrc_tmp = 1;
			MemtoReg_tmp = 1;
			RegWrite_tmp = 1;
			MemRead_tmp = 1;
			//MemRead1_tmp = 1;
			ALUOp_tmp = ALUadd;
		end
		
		lw: begin
			ALUSrc_tmp = 1;
			MemtoReg_tmp = 1;
			RegWrite_tmp = 1;
			MemRead_tmp = 1;
			//MemRead1_tmp = 1;
			//MemRead2_tmp = 1;
			//MemRead3_tmp = 1;
			ALUOp_tmp = ALUadd;
			isSigned_tmp = 1;
		end
		
		sb: begin
			ALUSrc_tmp = 1;
			//MemWrite0_tmp = 1;
			ALUOp_tmp = ALUadd;
			isSigned_tmp = 1;
		end
		
		sh: begin
			ALUSrc_tmp = 1;
			//MemWrite0_tmp = 1;
			//MemWrite1_tmp = 1;
			ALUOp_tmp = ALUadd;
			isSigned_tmp = 1;
		end
		
		sw: begin
			ALUSrc_tmp = 1;
			//MemWrite0_tmp = 1;
			//MemWrite1_tmp = 1;
			//MemWrite2_tmp = 1;
			//MemWrite3_tmp = 1;
			ALUOp_tmp = ALUadd;
			isSigned_tmp = 1;
		end
		
		beq: begin
			BranchBEQ_tmp = 1;
			ALUOp_tmp = ALUsub;
			isSigned_tmp = 1;
		end
		
		bne: begin
			BranchBNE_tmp = 1;
			ALUOp_tmp = ALUsub;
			isSigned_tmp = 1;
		end
		
		j: begin
			Jump_tmp = 1;
			isSigned_tmp = 1;
		end
		
		jal: begin
			Jump_tmp = 1;
			RegDst_tmp = 2'b10;
			RegWrite_tmp = 1;
			RegWriteSrc_tmp = 2'b10;
			isSigned_tmp = 1;
		end
	endcase
	
 end
 
 assign RegDst = RegDst_tmp;
 assign ALUSrc = ALUSrc_tmp;
 assign MemtoReg = MemtoReg_tmp;
 assign RegWrite = RegWrite_tmp;
 assign RegWriteSrc = RegWriteSrc_tmp;
 assign MemRead = MemRead_tmp;
 //assign MemRead1 = MemRead1_tmp;
 //assign MemRead2 = MemRead2_tmp;
 //assign MemRead3 = MemRead3_tmp;
 //assign MemWrite0 = MemWrite0_tmp;
 //assign MemWrite1 = MemWrite1_tmp;
 //assign MemWrite2 = MemWrite2_tmp;
 //assign MemWrite3 = MemWrite3_tmp;
 assign BranchBEQ = BranchBEQ_tmp;
 assign BranchBNE = BranchBNE_tmp;
 assign Jump = Jump_tmp;
 assign JumpRegID = JumpRegID_tmp;
 assign ALUOp = ALUOp_tmp;
 assign isSigned = isSigned_tmp;
 
endmodule



/*
DESCRIPTION
ALU

NOTES

TODO

*/

module alu_r0 #(
	parameter DATA_WIDTH = 32,
	parameter CTRL_WIDTH = 5,
	parameter STATUS_WIDTH = 4,
	parameter SHAMT_WIDTH = 5,
	parameter DELAY = 0
)(
	input clk,
	input rst,
	input en_n,
	input [DATA_WIDTH*2-1:0] dataIn,
	input [CTRL_WIDTH-1:0] ctrl,
	input [SHAMT_WIDTH-1:0] shamt,
	output [DATA_WIDTH-1:0] dataOut,
	output [STATUS_WIDTH-1:0] status
);
/**********
 *  Array Packing Defines 
 **********/
`define PACK_ARRAY(PK_WIDTH,PK_DEPTH,PK_SRC,PK_DEST, BLOCK_ID, GEN_VAR)    genvar GEN_VAR; generate for (GEN_VAR=0; GEN_VAR<(PK_DEPTH); GEN_VAR=GEN_VAR+1) begin: BLOCK_ID assign PK_DEST[((PK_WIDTH)*GEN_VAR+((PK_WIDTH)-1)):((PK_WIDTH)*GEN_VAR)] = PK_SRC[GEN_VAR][((PK_WIDTH)-1):0]; end endgenerate
`define UNPACK_ARRAY(PK_WIDTH,PK_DEPTH,PK_DEST,PK_SRC, BLOCK_ID, GEN_VAR)  genvar GEN_VAR; generate for (GEN_VAR=0; GEN_VAR<(PK_DEPTH); GEN_VAR=GEN_VAR+1) begin: BLOCK_ID assign PK_DEST[GEN_VAR][((PK_WIDTH)-1):0] = PK_SRC[((PK_WIDTH)*GEN_VAR+(PK_WIDTH-1)):((PK_WIDTH)*GEN_VAR)]; end endgenerate

/**********
 * Internal Signals
**********/
wire [DATA_WIDTH - 1:0] tmpIn [1:0];
reg [DATA_WIDTH - 1:0] outtmp;
wire [DATA_WIDTH - 1:0] addOut;
wire [DATA_WIDTH - 1:0] subOut;
wire [2*DATA_WIDTH - 1:0] multOut;
reg [STATUS_WIDTH - 1:0] statusTmp;
reg [DATA_WIDTH - 1:0] hi;
reg [DATA_WIDTH - 1:0] lo;
wire Cadd, Zadd, Vadd, Sadd, Csub, Zsub, Vsub, Ssub, Cmult, Zmult, Vmult, Smult;
/**********
 * Glue Logic 
 **********/
 `UNPACK_ARRAY(DATA_WIDTH,2,tmpIn,dataIn, U_BLK_0, idx_0)
 
/**********
 * Synchronous Logic
 **********/
/**********
 * Glue Logic 
 **********/
/**********
 * Components
 **********/
  delay #(
	.BIT_WIDTH(DATA_WIDTH),
	.DEPTH(1),
	.DELAY(DELAY)
 ) U_DEL(
	.clk(clk),
	.rst(rst),
	.en_n(en_n),
	.dataIn(outtmp),
	.dataOut(dataOut)
 );
 
 delay #(
	.BIT_WIDTH(STATUS_WIDTH),
	.DEPTH(1),
	.DELAY(DELAY)
 ) U_DEL2(
	.clk(clk),
	.rst(rst),
	.en_n(en_n),
	.dataIn(statusTmp),
	.dataOut(status)
 );
 
 add_r0 #(
	.DATA_WIDTH(DATA_WIDTH)
 )U_ADD(
	.input1(tmpIn[1]),
	.input2(tmpIn[0]),
	.dataOut(addOut),
	.C(Cadd),
	.Z(Zadd),
	.V(Vadd),
	.S(Sadd)
 );

 sub_r0 #(
	.DATA_WIDTH(DATA_WIDTH)
 )U_SUB(
	.input1(tmpIn[1]),
	.input2(tmpIn[0]),
	.dataOut(subOut),
	.C(Csub),
	.Z(Zsub),
	.V(Vsub),
	.S(Ssub)
 );
 
 mult_r0 #(
	.DATA_WIDTH(DATA_WIDTH)
 )U_MULT(
	.input1(tmpIn[1]),
	.input2(tmpIn[0]),
	.dataOut(multOut),
	.C(Cmult),
	.Z(Zmult),
	.V(Vmult),
	.S(Smult)
 );
 
/**********
 * Output Combinatorial Logic
 **********/
 always @(posedge clk) begin
	if(rst == 1'b1) begin
		hi <= {(DATA_WIDTH){1'b0}};
		lo <= {(DATA_WIDTH){1'b0}};
	end else if(ctrl == 5'b00010) begin
		hi <= multOut[2*DATA_WIDTH - 1:DATA_WIDTH];
		lo <= multOut[DATA_WIDTH-1:0];	
	end else if(ctrl == 5'b01011) begin	// mthi
		hi <= tmpIn[0];
	end else if(ctrl == 5'b01100) begin	// mtlo
		lo <= tmpIn[0];
	end
 end
 
 always @(dataIn, ctrl, shamt, addOut, subOut, multOut) begin
	statusTmp = {(STATUS_WIDTH){1'b0}};
 
	if(ctrl == 5'b00000) begin	// add, addu, addi, addiu
		outtmp = addOut;
		statusTmp[3] = Cadd;
		statusTmp[2] = Zadd;
		statusTmp[1] = Vadd;
		statusTmp[0] = Sadd;
	end else if(ctrl == 5'b00001) begin	// sub, subu, subi, subiu
		outtmp = subOut;
		statusTmp[3] = Csub;
		statusTmp[2] = Zsub;
		statusTmp[1] = Vsub;
		statusTmp[0] = Ssub;
	end else if(ctrl == 5'b00010) begin	// mult, multu	
		outtmp = multOut[DATA_WIDTH-1:0];
		statusTmp[3] = Cmult;
		statusTmp[2] = Zmult;
		statusTmp[1] = Vmult;
		statusTmp[0] = Smult;
	end else if(ctrl == 5'b00011) begin	// sll
		outtmp = tmpIn[0] << shamt;
	end else if(ctrl == 5'b00100) begin	// sllv
		outtmp = tmpIn[0] << tmpIn[1];
	end else if(ctrl == 5'b00101) begin	// srl
		outtmp = tmpIn[0] >> shamt;
	end else if(ctrl == 5'b00110) begin	// srlv
		outtmp = tmpIn[0] >> tmpIn[1];
	end else if(ctrl == 5'b00111) begin	// sra
		outtmp = $signed(tmpIn[0]) >>> shamt;
		//outtmp[DATA_WIDTH-1:DATA_WIDTH-shamt] = tmpIn[0][DATA_WIDTH-1];
	end else if(ctrl == 5'b01000) begin	// srav
		outtmp = $signed(tmpIn[0]) >>> tmpIn[1];
		//outtmp[DATA_WIDTH-1:DATA_WIDTH-shamt] = tmpIn[0][DATA_WIDTH-1];
	end else if(ctrl == 5'b01001) begin	// mfhi
		outtmp = hi;
	end else if(ctrl == 5'b01010) begin	// mflo
		outtmp = lo;
	end else if(ctrl == 5'b01011) begin	// mthi
		//hi <= tmpIn[0];
		outtmp = tmpIn[0];
	end else if(ctrl == 5'b01100) begin	// mtlo
		//lo <= tmpIn[0];
		outtmp = tmpIn[0];
	end else if(ctrl == 5'b01101) begin
		outtmp = tmpIn[0] & tmpIn[1];	// and, andi
	end else if(ctrl == 5'b01110) begin
		outtmp = tmpIn[0] | tmpIn[1];	// or, ori
	end else if(ctrl == 5'b01111) begin
		outtmp = tmpIn[0] ^ tmpIn[1];	// xor, xori
	end else if(ctrl == 5'b10000) begin
		outtmp = ~(tmpIn[0]) & ~(tmpIn[1]);	// nor
	end else if(ctrl == 5'b10001) begin
		if($signed(tmpIn[1]) < $signed(tmpIn[0])) begin	// slt
			outtmp = 32'h00000001;
		end else begin
			outtmp = 32'h00000000;
		end
	end else if(ctrl == 5'b10010) begin
		if($unsigned(tmpIn[1]) < $unsigned(tmpIn[0])) begin	// sltu
			outtmp = 32'h00000001;
		end else begin
			outtmp = 32'h00000000;
		end
	end
	
	if((ctrl != 5'b00000) && (ctrl != 5'b00001) && (ctrl != 5'b00010)) begin
		if(outtmp[DATA_WIDTH-1:0] == {(DATA_WIDTH){1'b0}}) begin
			statusTmp[2] = 1;
		end
		
		statusTmp[0] = outtmp[DATA_WIDTH-1];
	end
 end
endmodule




/*
DESCRIPTION
Pipelined MIPS Datapath

NOTES
	Stages:
	IF -> ID -> EXE -> MEM -> WB
TODO
*/

module datapath_r0 #(
	parameter DATA_WIDTH = 32,
	parameter ADDR_WIDTH = 5
)(
	input clk,
	input rst,
	input en_n
);

/**********
 * Internal Signals
**********/
wire [4:0] WriteReg; 					// Register to write to
wire JumpReg;

// IF
wire [DATA_WIDTH - 1:0] BranchOut;		// Output of Branch Mux
wire [DATA_WIDTH - 1:0] JumpOut;		// Output of Jump Mux
wire [DATA_WIDTH - 1:0] JumpRegOut;

reg  [DATA_WIDTH - 1:0] PC;			// PC
wire [DATA_WIDTH - 1:0] address;
wire [DATA_WIDTH - 1:0] PCPlus4;		// $PC + 4
wire [DATA_WIDTH - 1:0] instruction;

// IF/ID
wire [DATA_WIDTH - 1:0] IF_ID_PC;
wire [DATA_WIDTH - 1:0] IF_ID_PCPlus4;
wire [DATA_WIDTH - 1:0] IF_ID_Instruction;

// ID
wire isSigned;
wire Jump;
wire JumpRegID;
wire [1:0] RegDst;	// 0 = 20:16, 1 = 15:11, 2 = $31
wire ALUSrc;
wire [2:0] ALUOp;	// Input to the ALU Controller
wire [4:0] ALUCtrl;	// Input to the ALU
wire BranchBEQ;
wire BranchBNE;
wire MemtoReg;
wire ID_MemRead;
wire RegWrite;
wire [1:0] RegWriteSrc;	// 0 = output from memory, 1 = 16-bit left-shifted value for lui, 2 = PC + 4 for JAL

wire [2*DATA_WIDTH - 1:0] RegFileOut;	// 2 Outputs from the Reg file
wire equal;
wire [DATA_WIDTH - 1:0] SignExtOut;	// Output of Sign Extender

// Hazard Detection Unit
wire PCWrite;
wire ID_EX_CtrlFlush;
wire IF_ID_Flush;
wire IF_ID_Hold;

// ID/EX
wire ID_EX_ALUSrc;				// ALU Source Select
wire [2:0] ID_EX_ALUOp;				// ALU Operation
wire [1:0] ID_EX_RegDst;				// Destination Reg Select

wire ID_EX_BranchBEQ;			// Pass through to MEM
wire ID_EX_BranchBNE;			// Pass through to MEM
wire [5:0] ID_EX_Opcode;		// Pass through to MEM

wire ID_EX_MemtoReg;			// Pass through to MEM
wire ID_EX_MemRead;
wire ID_EX_RegWrite;			// Pass through to MEM
wire [1:0] ID_EX_RegWriteSrc;	// Pass through to MEM


wire [DATA_WIDTH - 1:0] ID_EX_PCPlus4;
wire [2*DATA_WIDTH - 1:0] ID_EX_RegFileOut;
wire [DATA_WIDTH - 1:0] ID_EX_SignExtOut;
wire [4:0] ID_EX_Instruction25to21;
wire [4:0] ID_EX_Instruction20to16;
wire [4:0] ID_EX_Instruction15to11;

// EX
wire [DATA_WIDTH - 1:0] ALUSrcOut;	// Output of ALU Source Mux
wire [DATA_WIDTH - 1:0] ALUOut;	// Output of ALU
wire [3:0] StatusReg;			// Status Register from ALU

// EX/MEM
wire EX_MEM_BranchBEQ;			// if BEQ
wire EX_MEM_BranchBNE;			// if BNE
wire [3:0] EX_MEM_StatusReg;			// status reg for use with branch, etc...
wire [5:0] EX_MEM_Opcode;		// Load or store opcode for Memory Controller

wire EX_MEM_MemtoReg;			// Pass through to WB
wire EX_MEM_RegWrite;			// Pass through to WB
wire [1:0] EX_MEM_RegWriteSrc;	// Pass through to WB

wire [DATA_WIDTH - 1:0] EX_MEM_BranchADD;			// Result of addition in EX stage
wire [DATA_WIDTH - 1:0] EX_MEM_PCPlus4;
wire [DATA_WIDTH - 1:0] EX_MEM_SignExtOut;
wire [DATA_WIDTH - 1:0] EX_MEM_ALUOut;
wire [DATA_WIDTH - 1:0] EX_MEM_ReadData2;
wire [ADDR_WIDTH - 1:0] EX_MEM_WriteReg;

// MEM
wire [5:0] ALUaddress;
wire [1:0] MemSelect;

wire [3:0] MemRead;
wire [3:0] MemWrite;
wire MemMux1Sel;
wire [1:0] MemMux2Sel;
wire [1:0] MemMux3Sel;
wire [7:0] MemMux1Out;
wire [7:0] MemMux2Out;
wire [7:0] MemMux3Out;

wire [DATA_WIDTH - 1:0] DataMemOut;		// Output from Data Memory
wire [DATA_WIDTH - 1:0] MemOut;

// MEM/WB
wire MEM_WB_MemtoReg;			// Choose between Memory Out or ALU Result
wire MEM_WB_RegWrite;			// Write enable for Reg File
wire [1:0] MEM_WB_RegWriteSrc;	// Choose between output of MemtoReg, Immediate Value, or $PC+4

wire [DATA_WIDTH - 1:0] MEM_WB_PCPlus4;
wire [DATA_WIDTH - 1:0] MEM_WB_SignExtOut;
wire [DATA_WIDTH - 1:0] MEM_WB_DataMemOut;
wire [DATA_WIDTH - 1:0] MEM_WB_MemOut;
wire [DATA_WIDTH - 1:0] MEM_WB_ALUOut;
wire [ADDR_WIDTH - 1:0] MEM_WB_WriteReg;

// WB
wire [DATA_WIDTH - 1:0] MemtoRegOut;	// Out of MemtoReg Mux
wire [DATA_WIDTH - 1:0] WriteData;		// Data Written to Reg file

// Forwarding Unit
wire [1:0] ForwardA;
wire [1:0] ForwardB;
wire [DATA_WIDTH - 1:0] ForwardAOut;
wire [DATA_WIDTH - 1:0] ForwardBOut;

/**********
 * Glue Logic 
 **********/
/**********
 * Synchronous Logic
 **********/
/**********
 * Glue Logic 
 **********/
/**********
 * Components
 **********/
 // ------- Forwarding Components ------ //
 forwardingUnit_r0 #(
	.BIT_WIDTH(ADDR_WIDTH)
 ) U_FWDUNIT (
	.ID_EX_Rs(ID_EX_Instruction25to21),
	.ID_EX_Rt(ID_EX_Instruction20to16),
	.EX_MEM_Rd(EX_MEM_WriteReg),
	.MEM_WB_Rd(MEM_WB_WriteReg),
	.EX_MEM_RegWrite(EX_MEM_RegWrite),
	.MEM_WB_RegWrite(MEM_WB_RegWrite),
	.ForwardA(ForwardA),
	.ForwardB(ForwardB)
 );
 
 mux #(
	.BIT_WIDTH(DATA_WIDTH),
	.DEPTH(3)
 ) U_FWDA (
	.dataIn({EX_MEM_ALUOut, MemtoRegOut, ID_EX_RegFileOut[2*DATA_WIDTH - 1:DATA_WIDTH]}),
	.sel(ForwardA),
	.dataOut(ForwardAOut)
 );
 
 mux #(
	.BIT_WIDTH(DATA_WIDTH),
	.DEPTH(3)
 ) U_FWDB (
	.dataIn({EX_MEM_ALUOut, MemtoRegOut, ID_EX_RegFileOut[DATA_WIDTH - 1:0]}),
	.sel(ForwardB),
	.dataOut(ForwardBOut)
 );
 
 // ----------- Hazard Detection Unit ------- //
 hazardDetectionUnit_r0 U_HDU(
	.IF_ID_Opcode(IF_ID_Instruction[31:26]),
	.IF_ID_Funcode(IF_ID_Instruction[5:0]),
	.IF_ID_Rs(IF_ID_Instruction[25:21]),
	.IF_ID_Rt(IF_ID_Instruction[20:16]),
	.ID_EX_MemRead(ID_EX_MemRead),
	.ID_EX_Rt(ID_EX_Instruction20to16),
	.equal(equal),
	.ID_EX_Rd(WriteReg),
	.EX_MEM_Rd(EX_MEM_WriteReg),
	.ID_EX_RegWrite(ID_EX_RegWrite),
	.EX_MEM_RegWrite(EX_MEM_RegWrite),
	.PCWrite(PCWrite),
	.ID_EX_CtrlFlush(ID_EX_CtrlFlush),
	.IF_ID_Flush(IF_ID_Flush),
	.IF_ID_Hold(IF_ID_Hold)
 );
 
 // --------- PIPELINE REGS ------------ //
 // IF/ID Register
 delay #(
	.BIT_WIDTH(DATA_WIDTH),
	.DEPTH(3),
	.DELAY(1)
 ) U_IF_ID_REG (
	.clk(clk),
	.rst(IF_ID_Flush | rst),
	.en_n(IF_ID_Hold),
	.dataIn({PC, PCPlus4, instruction}),
	.dataOut({IF_ID_PC, IF_ID_PCPlus4, IF_ID_Instruction})
 );
 
 // ID/EX Register
 // --- CONTROL PIPELINE REGS --- //
 delay #(
	.BIT_WIDTH(1),
	.DEPTH(6),
	.DELAY(1)
 ) U_ID_EX_REG0 (
	.clk(clk),
	.rst(ID_EX_CtrlFlush | rst),
	.en_n(1'b0),
	.dataIn({ALUSrc, BranchBEQ, BranchBNE, MemtoReg, ID_MemRead, RegWrite}),
	.dataOut({ID_EX_ALUSrc, ID_EX_BranchBEQ, ID_EX_BranchBNE, ID_EX_MemtoReg, ID_EX_MemRead, ID_EX_RegWrite})
 );
 
  delay #(
	.BIT_WIDTH(2),
	.DEPTH(2),
	.DELAY(1)
 ) U_ID_EX_REG1 (
	.clk(clk),
	.rst(ID_EX_CtrlFlush | rst),
	.en_n(1'b0),
	.dataIn({RegDst, RegWriteSrc}),
	.dataOut({ID_EX_RegDst, ID_EX_RegWriteSrc})
 );
 
  delay #(
	.BIT_WIDTH(3),
	.DEPTH(1),
	.DELAY(1)
 ) U_ID_EX_REG2 (
	.clk(clk),
	.rst(ID_EX_CtrlFlush | rst),
	.en_n(1'b0),
	.dataIn(ALUOp),
	.dataOut(ID_EX_ALUOp)
 );
 
 // --- END CONTROL PIPELINE REGS --- //
 
  delay #(
	.BIT_WIDTH(6),
	.DEPTH(1),
	.DELAY(1)
 ) U_ID_EX_REG3 (
	.clk(clk),
	.rst(rst),
	.en_n(1'b0),
	.dataIn(IF_ID_Instruction[31:26]),
	.dataOut(ID_EX_Opcode)
 );
 
 delay #(
	.BIT_WIDTH(DATA_WIDTH),
	.DEPTH(4),
	.DELAY(1)
 ) U_ID_EX_REG4 (
	.clk(clk),
	.rst(rst),
	.en_n(1'b0),
	.dataIn({IF_ID_PCPlus4, RegFileOut, SignExtOut}),
	.dataOut({ID_EX_PCPlus4, ID_EX_RegFileOut, ID_EX_SignExtOut})
 );
 
 delay #(
	.BIT_WIDTH(ADDR_WIDTH),
	.DEPTH(3),
	.DELAY(1)
 ) U_ID_EX_REG5 (
	.clk(clk),
	.rst(rst),
	.en_n(1'b0),
	.dataIn({IF_ID_Instruction[25:21], IF_ID_Instruction[20:16], IF_ID_Instruction[15:11]}),
	.dataOut({ID_EX_Instruction25to21, ID_EX_Instruction20to16, ID_EX_Instruction15to11})
 );
 
 // EX/MEM Register
 delay #(
	.BIT_WIDTH(1),
	.DEPTH(4),
	.DELAY(1)
 ) U_EX_MEM_REG0 (
	.clk(clk),
	.rst(rst),
	.en_n(1'b0),
	.dataIn({ID_EX_BranchBEQ, ID_EX_BranchBNE, ID_EX_MemtoReg, ID_EX_RegWrite}),
	.dataOut({EX_MEM_BranchBEQ, EX_MEM_BranchBNE, EX_MEM_MemtoReg, EX_MEM_RegWrite})
 );
 
 delay #(
	.BIT_WIDTH(2),
	.DEPTH(1),
	.DELAY(1)
 ) U_EX_MEM_REG1 (
	.clk(clk),
	.rst(rst),
	.en_n(1'b0),
	.dataIn(ID_EX_RegWriteSrc),
	.dataOut(EX_MEM_RegWriteSrc)
 );
 
  delay #(
	.BIT_WIDTH(4),
	.DEPTH(1),
	.DELAY(1)
 ) U_EX_MEM_REG2 (
	.clk(clk),
	.rst(rst),
	.en_n(1'b0),
	.dataIn(StatusReg),
	.dataOut(EX_MEM_StatusReg)
 );
 
  delay #(
	.BIT_WIDTH(6),
	.DEPTH(1),
	.DELAY(1)
 ) U_EX_MEM_REG3 (
	.clk(clk),
	.rst(rst),
	.en_n(1'b0),
	.dataIn(ID_EX_Opcode),
	.dataOut(EX_MEM_Opcode)
 );
 
  delay #(
	.BIT_WIDTH(DATA_WIDTH),
	.DEPTH(5),
	.DELAY(1)
 ) U_EX_MEM_REG4 (
	.clk(clk),
	.rst(rst),
	.en_n(1'b0),
	.dataIn({{ID_EX_SignExtOut[29:0], 2'b00} + ID_EX_PCPlus4, ID_EX_PCPlus4, ID_EX_SignExtOut, ALUOut, ForwardBOut}),
	.dataOut({EX_MEM_BranchADD, EX_MEM_PCPlus4, EX_MEM_SignExtOut, EX_MEM_ALUOut, EX_MEM_ReadData2})
 );
 
   delay #(
	.BIT_WIDTH(ADDR_WIDTH),
	.DEPTH(1),
	.DELAY(1)
 ) U_EX_MEM_REG5 (
	.clk(clk),
	.rst(rst),
	.en_n(1'b0),
	.dataIn(WriteReg),
	.dataOut(EX_MEM_WriteReg)
 );
 
 // MEM/WB Register
 delay #(
	.BIT_WIDTH(1),
	.DEPTH(2),
	.DELAY(1)
 ) U_MEM_WB_REG0 (
	.clk(clk),
	.rst(rst),
	.en_n(1'b0),
	.dataIn({EX_MEM_MemtoReg, EX_MEM_RegWrite}),
	.dataOut({MEM_WB_MemtoReg, MEM_WB_RegWrite})
 );
 
  delay #(
	.BIT_WIDTH(2),
	.DEPTH(1),
	.DELAY(1)
 ) U_MEM_WB_REG1 (
	.clk(clk),
	.rst(rst),
	.en_n(1'b0),
	.dataIn(EX_MEM_RegWriteSrc),
	.dataOut(MEM_WB_RegWriteSrc)
 );
 
 delay #(
	.BIT_WIDTH(DATA_WIDTH),
	.DEPTH(5),
	.DELAY(1)
 ) U_MEM_WB_REG2 (
	.clk(clk),
	.rst(rst),
	.en_n(1'b0),
	.dataIn({EX_MEM_PCPlus4, EX_MEM_SignExtOut, DataMemOut, MemOut, EX_MEM_ALUOut}),
	.dataOut({MEM_WB_PCPlus4, MEM_WB_SignExtOut, MEM_WB_DataMemOut, MEM_WB_MemOut, MEM_WB_ALUOut})
 );
 
 delay #(
	.BIT_WIDTH(ADDR_WIDTH),
	.DEPTH(1),
	.DELAY(1)
 ) U_MEM_WB_REG3 (
	.clk(clk),
	.rst(rst),
	.en_n(1'b0),
	.dataIn(EX_MEM_WriteReg),
	.dataOut(MEM_WB_WriteReg)
 );
 
 
 // ----- INSTRUCTION FETCH (IF) ----- //
  mux #(
	.BIT_WIDTH(DATA_WIDTH),
	.DEPTH(2)
 )U_BRANCHMUX(
	.dataIn({{SignExtOut[29:0], 2'b00} + IF_ID_PCPlus4, PCPlus4}),
	.sel((BranchBEQ & equal) | (BranchBNE & ~equal)),
	.dataOut(BranchOut)
 );
 
  mux #(
	.BIT_WIDTH(DATA_WIDTH),
	.DEPTH(2)
 )U_JUMPMUX(
	.dataIn({{IF_ID_PCPlus4[31:28], {IF_ID_Instruction[25:0], 2'b00}}, BranchOut}),
	.sel(Jump),
	.dataOut(JumpOut)
 );
 
  mux #(
	.BIT_WIDTH(DATA_WIDTH),
	.DEPTH(2)
 )U_JUMPREGMUX(
	.dataIn({RegFileOut[2*DATA_WIDTH - 1:DATA_WIDTH], JumpOut}),
	.sel(JumpRegID),
	.dataOut(JumpRegOut)
 );
 
  rom U_rom(
	.q(instruction),
	.a(address[6:0])
 );
 
 // ----- INSTRUCTION DECODE (ID) ----- //
  controller_r0 U_CONTROLLER(
	.opcode(IF_ID_Instruction[31:26]),
	.funcode(IF_ID_Instruction[5:0]),
	.RegDst(RegDst),
	.ALUSrc(ALUSrc),
	.MemtoReg(MemtoReg),
	.MemRead(ID_MemRead),
	.RegWrite(RegWrite),
	.RegWriteSrc(RegWriteSrc),
	.Jump(Jump),
	.JumpRegID(JumpRegID),
	.BranchBEQ(BranchBEQ),
	.BranchBNE(BranchBNE),
	.ALUOp(ALUOp),
	.isSigned(isSigned)
 );
 
  registerFile #(
	.DATA_WIDTH(DATA_WIDTH),
	.RD_DEPTH(2),
	.REG_DEPTH(32),
	.ADDR_WIDTH(ADDR_WIDTH)
 )U_REGFILE(
	.clk(clk),
	.rst(rst),
	.wr(MEM_WB_RegWrite),
	.rr({IF_ID_Instruction[25:21], IF_ID_Instruction[20:16]}),
	.rw(MEM_WB_WriteReg),
	.d(WriteData),
	.q(RegFileOut)
 );
 
 comparator_r0 #(
	.BIT_WIDTH(DATA_WIDTH)
 ) U_COMPARE(
	.dataIn(RegFileOut),
	.equal(equal)
 );
 
  signextender_r0 #(
	.IN_WIDTH(16),
	.OUT_WIDTH(DATA_WIDTH),
	.DEPTH(1),
	.DELAY(0)
 )U_SIGNEXTENDER(
	.clk(clk),
	.rst(rst),
	.en_n(en_n),
	.dataIn(IF_ID_Instruction[15:0]),
	.dataOut(SignExtOut),
	.isSigned(isSigned)
 );
 
 // ----- EXECUTE (EX) ----- //
  mux #(
	.BIT_WIDTH(DATA_WIDTH),
	.DEPTH(2)
 )U_ALUSRCMUX(
	//.dataIn({ID_EX_SignExtOut, ID_EX_RegFileOut[DATA_WIDTH - 1:0]}),
	.dataIn({ID_EX_SignExtOut, ForwardBOut}),
	.sel(ID_EX_ALUSrc),
	.dataOut(ALUSrcOut)
 );
 
  alu_controller_r0 U_ALUCONTROLLER(
	.ALUOp(ID_EX_ALUOp),
	.funcode(ID_EX_SignExtOut[5:0]),
	.ALUCtrl(ALUCtrl),
	.JumpReg(JumpReg)
 );
 
  alu_r0 #(
	.DATA_WIDTH(DATA_WIDTH),
	.CTRL_WIDTH(5),
	.STATUS_WIDTH(4),
	.SHAMT_WIDTH(5),
	.DELAY(0)
 )U_ALU(
	.clk(clk),
	.rst(rst),
	.en_n(en_n),
	.dataIn({ForwardAOut, ALUSrcOut}),
	.ctrl(ALUCtrl),
	.shamt(ID_EX_SignExtOut[10:6]),
	.dataOut(ALUOut),
	.status(StatusReg)
 );
 
  mux #(
	.BIT_WIDTH(ADDR_WIDTH),
	.DEPTH(3)
 )U_REGDSTMUX(
	.dataIn({5'b11111, ID_EX_Instruction15to11, ID_EX_Instruction20to16}),
	.sel(ID_EX_RegDst),
	.dataOut(WriteReg)
 );
 
 // ----- MEMORY (MEM) ----- //
 // ----- Memory Muxes ----- //
 mux #(
	.BIT_WIDTH(8),
	.DEPTH(2)
 ) U_MEMMUX1(
	.dataIn({EX_MEM_ReadData2[15:8], EX_MEM_ReadData2[7:0]}),
	.sel(MemMux1Sel),
	.dataOut(MemMux1Out)
 );
 
 mux #(
	.BIT_WIDTH(8),
	.DEPTH(3)
 ) U_MEMMUX2(
	.dataIn({EX_MEM_ReadData2[23:16], EX_MEM_ReadData2[15:8], EX_MEM_ReadData2[7:0]}),
	.sel(MemMux2Sel),
	.dataOut(MemMux2Out)
 );
 
 mux #(
	.BIT_WIDTH(8),
	.DEPTH(3)
 ) U_MEMMUX3(
	.dataIn({EX_MEM_ReadData2[31:24], EX_MEM_ReadData2[15:8], EX_MEM_ReadData2[7:0]}),
	.sel(MemMux3Sel),
	.dataOut(MemMux3Out)
 );
 
 // ---- Data Memory ---- //
 ram U_ram0(
	.q(DataMemOut[7:0]),
	.d(EX_MEM_ReadData2[7:0]),
	.a(ALUaddress),
	.rst(rst),
	.we(MemWrite[0]),
	.re(MemRead[0]),
	.clk(clk)
 );
 
 ram U_ram1(
	.q(DataMemOut[15:8]),
	.d(MemMux1Out),
	.a(ALUaddress),
	.rst(rst),
	.we(MemWrite[1]),
	.re(MemRead[1]),
	.clk(clk)
 );
 
 ram U_ram2(
	.q(DataMemOut[23:16]),
	.d(MemMux2Out),
	.a(ALUaddress),
	.rst(rst),
	.we(MemWrite[2]),
	.re(MemRead[2]),
	.clk(clk)
 );
 
 ram U_ram3(
	.q(DataMemOut[31:24]),
	.d(MemMux3Out),
	.a(ALUaddress),
	.rst(rst),
	.we(MemWrite[3]),
	.re(MemRead[3]),
	.clk(clk)
 );
 
  memout_r0 # (
	.DATA_WIDTH(8)
 ) U_MEMOUT (
	.Mem0Out(DataMemOut[7:0]),
	.Mem1Out(DataMemOut[15:8]),
	.Mem2Out(DataMemOut[23:16]),
	.Mem3Out(DataMemOut[31:24]),
	.MemSel(MemSelect),
	.Opcode(EX_MEM_Opcode),
	.MemOut(MemOut)
 );
 
 memcontroller_r0 U_MEMCONTROLLER(
	.opcode(EX_MEM_Opcode),
	.MemSelect(MemSelect),
	.MemWrite(MemWrite),
	.MemRead(MemRead),
	.MemMux1Sel(MemMux1Sel),
	.MemMux2Sel(MemMux2Sel),
	.MemMux3Sel(MemMux3Sel)
 );
 
 // ----- WWRITE BACK (WB) ----- //
  mux #(
	.BIT_WIDTH(DATA_WIDTH),
	.DEPTH(2)
 )U_MEMTOREGMUX(
	.dataIn({MEM_WB_MemOut, MEM_WB_ALUOut}),
	.sel(MEM_WB_MemtoReg),
	.dataOut(MemtoRegOut)
 );
 
  mux #(
	.BIT_WIDTH(DATA_WIDTH),
	.DEPTH(3)
 )U_REGWRITESRCMUX(
	.dataIn({MEM_WB_PCPlus4, {MEM_WB_SignExtOut[15:0], 16'h0000}, MemtoRegOut}),
	.sel(MEM_WB_RegWriteSrc),
	.dataOut(WriteData)
 );
  
/**********
 * Output Combinatorial Logic
 **********/
 assign PCPlus4 = PC + 4;
 assign address = PC >> 2;	// Shift PC by two since ROM is byte-addressable
 
 // ------------------ Memory Signals --------------------- //
 assign ALUaddress = EX_MEM_ALUOut[7:2];	// Calculated Address bits
 assign MemSelect = EX_MEM_ALUOut[1:0];	// Select bits from ALU output
 
 always @(posedge clk) begin
	if(rst == 1'b1) begin
		PC <= {(DATA_WIDTH){1'b0}};
	end else begin
		if(PCWrite) begin
			PC <= JumpRegOut;
		end
	end
	// $display("PC : %d", PC);
	$display("ALU address : %h" ,ALUaddress) ; 
	
	$display("Write data : %h" ,WriteData) ; 
 end
endmodule

