//=======================================================
//  Display Driver Top Level
//  Team 253XY
//=======================================================

`define HEIGHT    16'd480 // height of image in pixels
`define TILE_SIZE 16'd48 // size of one maze square in pixels

module display_driver(
	//////////// CLOCK //////////
	input         CLOCK_50, // 50 MHz clock

	//////////// LED //////////
	output [7:0]  LED,     // leds for debugging

	//////////// KEY //////////
	input  [1:0]  KEY,    // buttons for reset and debugging

	//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
	output [33:0] GPIO_0, // output pins

	//////////// GPIO_1, GPIO_1 connect to GPIO Default //////////
	input  [33:0] GPIO_1 // input pins
);

//=======================================================
//  REG/WIRE declarations
//=======================================================

wire        rst;                 // reset
wire        clk;                 // clock
wire [15:0] pixel_index;         // scanning point position within a line
wire [15:0] line_index;          // scanning line position
wire        blanking;            // blanking signal, asserted during porches and sync pulses
wire        h_sync;              // horizontal sync signal
wire        v_sync;              // vertical sync signal

wire [9:0]  parallel_port_async; // raw parallel port input
wire [9:0]  parallel_port_sync;  // parallel port input synchronized to the FPGA clock
wire        arduino_clk;         // synchronized parallel port clock
wire        new_frame;           // synchronized new transmission signal
wire [7:0]  data_in;             // synchronized parallel port data

wire [7:0]  dec_out;             // tile decoder output

reg  [7:0]  maze [9:0][13:0];    // maze data memory. 0 -> 13 in order to cover the entire rectangular width
reg  [7:0]  data_index;          // byte index within a transmission
reg         prev_arduino_clk;    // previous state of the parallel port clock

//=======================================================
//  Structural coding
//=======================================================

assign rst = !KEY[0]; // assign reset to KEY 0
assign parallel_port_async = {GPIO_1[23], GPIO_1[21], GPIO_1[19], GPIO_1[17], GPIO_1[13], GPIO_1[11], GPIO_1[9], GPIO_1[8], GPIO_1[10], GPIO_1[12]}; // assign parallel port pins
assign {GPIO_0[9], GPIO_0[11], GPIO_0[13], GPIO_0[15], GPIO_0[17], GPIO_0[19], GPIO_0[21], GPIO_0[23]} = dec_out; // assign color output pins
assign GPIO_0[7] = h_sync; // assign horizontal sync output pin
assign GPIO_0[5] = v_sync; // assign vertical sync output pin

assign arduino_clk = parallel_port_sync[8]; // separate clock from synchronized parallel port
assign new_frame = parallel_port_sync[9]; // separate new transmission signal from synchronized parallel port
assign data_in = parallel_port_sync[7:0]; // separate data from synchronized parallel port

// synchronous process for updating the maze memory with new data
always @(posedge clk) begin
	if(rst) begin // if reset is pressed...
		// variables used to reset the tile states
		integer x_val;
		integer y_val;

		for(y_val = 0; y_val < 10; y_val = y_val + 1)
		begin
			for(x_val = 0; x_val < 14; x_val = x_val + 1)
			begin
				maze[y_val][x_val] <= 'b00000000; // reset all data to zeros (unexplored)
			end
		end

		data_index <= 'd0; // reset the byte index

		prev_arduino_clk <= 'b0; // reset the previous clock state
	end
	else begin // if reset is not pressed...
		if(arduino_clk == 'b1 && prev_arduino_clk == 'b0) begin // if there's a rising edge on the parallel port clock...
			maze[data_index / 10][data_index % 10] <= data_in; // update the corresponding tile using the new received byte

			if(new_frame == 'b1) data_index <= 'd1; // if new transmission is asserted, next byte should have index 1. prevents synchronization errors caused by lost bytes
			else if(data_index == 'd99) data_index <= 'd0; // if this is the last byte in the transmission, the next should have index 0
			else data_index <= data_index + 8'd1; // otherwise the index should be incremented
		end

		prev_arduino_clk <= arduino_clk; // retain previous state of parallel port clock in order to detect rising edges
	end
end

// pll for generating a 25.17 MHz pixel clock
vga_driver_pll clk_gen(
	.inclk0(CLOCK_50),
	.c0(clk)
);

// vga driver
vga_driver vga(
	.rst(rst),
	.clk(clk),
	.pixel_index(pixel_index),
	.line_index(line_index),
	.blanking(blanking),
	.h_sync(h_sync),
	.v_sync(v_sync)
);

// parallel port synchronizer needed to prevent metastability
synchronizer arduino_sync(
	.rst(rst),
	.clk(clk),
	.async(parallel_port_async),
	.sync(parallel_port_sync)
);

// decoder module. takes in maze data and scanning point position and outputs the corresponding pixel color
tile_decoder dec(
	.rst(rst),
	.clk(clk),
	.in(maze[(`HEIGHT - line_index - 16'd1) / `TILE_SIZE][(pixel_index) / `TILE_SIZE]),
	.x(pixel_index % `TILE_SIZE),
	.y(line_index % `TILE_SIZE),
	.blanking(blanking),
	.out(dec_out)
);

endmodule
