/* Description
 Testbench for the datapath
*/
`timescale 1ns/1ns
`include "merged.v"

module datapath_r0_tb;
//Parameters declaration: 
defparam UUT.DATA_WIDTH = 32;
parameter DATA_WIDTH = 32;
defparam UUT.ADDR_WIDTH = 5;
parameter ADDR_WIDTH = 5;


reg done = 0;

//Internal signals declarations:
reg clk=0;
reg rst;
reg en_n=1;
always #1 clk = ~clk; 
	

// initial
// 	$monitor($realtime,,"ps %h %h %h ",clk,rst,en_n);
	
	
initial begin
	$dumpfile("datapath.vcd") ;
    $dumpvars(0,datapath_r0_tb) ;
	done = 0;
	rst = 1;
	//PC_we = 1;
	#5;
	
	rst = 0;
	#25;
	
	// done = 1;
	// # 5;
	$finish ; 
end

/**********
 * Components
 **********/   
 
 datapath_r0 #(
	.DATA_WIDTH(DATA_WIDTH),
	.ADDR_WIDTH(ADDR_WIDTH)
 )UUT(
	.clk(clk),
	.rst(rst),
	.en_n(en_n)
	//.PC_we(PC_we)
 );


endmodule
