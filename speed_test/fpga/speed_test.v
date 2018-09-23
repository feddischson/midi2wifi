module speed_test(

	input 		          		FPGA_CLK1_50,
	input 		          		FPGA_CLK2_50,
	input 		          		FPGA_CLK3_50,
	input 		     [1:0]		KEY,
	output		     [7:0]		LED,
	input 		     [3:0]		SW,
    input            [9:0]     GPIO
);

localparam LATCH_CNT_SEL = 4; 

// internal reset
wire reset;

reg [10:0] latch;
reg [10:0] latch_d;

// 34 bit ~ 8 bit + log( 50e6 )
// The most sigificant 8 bit will be used as output, which is updated
// every 1.34 (2^26/50e6) seconds if SW = 4'b0001
reg [33:0] cnt;


// create high-active reset from both low-active inputs 
assign reset = ~&KEY;

// assign the 7 MSBs from counter to the LED
assign LED   = {cnt[33:26], ^GPIO, ^latch};




always @(posedge FPGA_CLK1_50)
  if( reset ) begin
    cnt <= 0;
    latch_d <= 0;
    latch   <= 0;
  end else begin
    cnt <= cnt + SW;
    latch_d[10:0] <= cnt[10:0];
    latch[10:0] = latch_d[10:0] & (~cnt[10:0]);
  end
endmodule
