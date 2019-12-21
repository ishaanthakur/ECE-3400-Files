//=======================================================
//  Tile Decoder Module
//  Team 253XY
//=======================================================

// bit structure of byte that represents one node
`define VISITED        7
`define CURRENT        6
`define TARGET         5
`define NEXT           4
`define WALL_DOWN      3
`define WALL_LEFT      2
`define TEMP_WALL_DOWN 1
`define TEMP_WALL_LEFT 0

`define TILE_SIZE      16'd48 // size of one maze square in pixels
`define WALL_WIDTH     16'd5  // wall thickness in pixels

module tile_decoder(
	input            rst,
	input            clk,
	input      [7:0] in,
	input      [7:0] x,
	input      [7:0] y,
	input            blanking,
	output reg [7:0] out
);

//=======================================================
//  Parameter declarations
//=======================================================

// dac inputs for various colors
localparam RED     = 8'b001_000_00;
localparam MAGENTA = 8'b001_000_01;
localparam YELLOW  = 8'b001_001_00;
localparam WHITE   = 8'b001_001_01;
localparam BLACK   = 8'b000_000_00;
localparam BLUE    = 8'b000_000_01;
localparam GREEN   = 8'b000_001_00;

//=======================================================
//  REG/WIRE declarations
//=======================================================

reg [7:0] n_out;     // next color output
reg [7:0] bkg_color; // background color of a tile

//=======================================================
//  Structural coding
//=======================================================

// synchronous process to implement output register
always @(posedge clk) begin
	if(rst) out <= 8'b0; // reset the color to black upon reset
	else out <= blanking ? 8'b0 : n_out; // otherwise output the color if blanking is not asserted
end

// combinational process for decoding maze into pixel colors
always @(*) begin
	if(in[`TARGET]) bkg_color = RED; // if the tile is the target, make its backround red
	else if (in[`NEXT]) bkg_color = MAGENTA; // otherwise, if it's the next step, make its background magenta
	else if (in[`CURRENT]) bkg_color = YELLOW; // otherwise, if it's the current robot position, make its background yellow
	else if (in[`VISITED]) bkg_color = WHITE; // otherwise, if it's visited, make its background white
	else bkg_color = BLACK; // otherwise, leave it black

	if((x <= `WALL_WIDTH && in[`WALL_LEFT]) || (y + `WALL_WIDTH >= `TILE_SIZE && in[`WALL_DOWN])) // if there's a wall at this scanning position...
	begin
		n_out = BLUE; // output blue
	end
	else if ((x <= `WALL_WIDTH && in[`TEMP_WALL_LEFT]) || (y + `WALL_WIDTH >= `TILE_SIZE && in[`TEMP_WALL_DOWN])) begin // otherwise, if there's a temporary wall...
		n_out = GREEN; // output green
	end
	else n_out = bkg_color; // otherwise output the background color
end
endmodule

