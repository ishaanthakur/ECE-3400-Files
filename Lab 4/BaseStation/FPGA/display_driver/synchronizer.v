//=======================================================
//  10 Bit Synchronizer Module
//  Implemented using two stage shift registers
//  Team 253XY
//=======================================================

module synchronizer(
	input            rst,
	input            clk,
	input      [9:0] async,
	output reg [9:0] sync
);

//=======================================================
//  REG/WIRE declarations
//=======================================================

reg [9:0] input_reg; // first stage

//=======================================================
//  Structural coding
//=======================================================

// synchronous process for implementing the shift registers
always @(posedge clk) begin
	if(rst) begin // if reset is asserted...
		// reset both stages to zero
		input_reg = 'b0000000000;
		sync = 'b0000000000;
	end
	else begin
		// otherwise shift the values through the two stages
		input_reg <= async; // raw input to first stage
		sync <= input_reg; // first stage to second stage
	end
end
endmodule
