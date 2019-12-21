//=======================================================
//  VGA Driver Module
//  Team 253XY
//=======================================================

`define H_FRONT_PORCH_LEN 16'd16  // duration of horizontal front porch in pixels
`define H_SYNC_LEN        16'd96  // duration of horizonal sync pulse in pixels
`define H_BACK_PORCH_LEN  16'd48  // duration of horizontal back porch in pixels
`define WIDTH             16'd640 // image width in pixels

`define V_FRONT_PORCH_LEN 16'd10  // duration of vertical front porch in lines
`define V_SYNC_LEN        16'd2   // duration of vertical sync pulse in lines
`define V_BACK_PORCH_LEN  16'd33  // duration of vertical back porch in lines
`define HEIGHT            16'd480 // image height in lines

module vga_driver(
	input             rst,
	input             clk,
	output reg [15:0] pixel_index,
	output reg [15:0] line_index,
	output reg        blanking,
	output reg        h_sync,
	output reg        v_sync
);

//=======================================================
//  Parameter declarations
//=======================================================

localparam FRONT_PORCH = 2'd0; // front porch state index
localparam SYNC        = 2'd1; // sync pulse state index
localparam BACK_PORCH  = 2'd2; // back porch state index
localparam ACTIVE      = 2'd3; // displaying image state index

//=======================================================
//  REG/WIRE declarations
//=======================================================

reg  [1:0] h_state;        // horizonatal FSM state
reg  [1:0] n_h_state;      // next horizontal FSM state
reg  [1:0] v_state;        // vertical FSM state
reg  [1:0] n_v_state;      // next vertical FSM state
reg  [15:0] n_pixel_index; // next pixel index
reg  [15:0] n_line_index;  // next line index

//=======================================================
//  Structural coding
//=======================================================

// synchronous process for horizontal and vertical FSMs
always @(posedge clk) begin
	if(rst) begin // upon reset...
		// go to the front porch state on both FSMs
		h_state <= FRONT_PORCH;
		v_state <= FRONT_PORCH;

		// set the indexes to zero
		pixel_index <= 'd0;
		line_index  <= 'd0;
	end
	else begin // otherwise set the registers to the new values
		h_state <= n_h_state;
		v_state <= n_v_state;
		pixel_index <= n_pixel_index;
		line_index  <= n_line_index;
	end
end

// combinational process for the FSMs
always @(*) begin
	case(h_state) // horizontal FSM
		FRONT_PORCH:
		n_h_state = (pixel_index + 16'd1 == `H_FRONT_PORCH_LEN) ? SYNC : h_state; // switch to SYNC state if H_FRONT_PORCH_LEN time elapsed

		SYNC:
		n_h_state = (pixel_index + 16'd1 == `H_SYNC_LEN) ? BACK_PORCH : h_state; // switch to BACK_PORCH state if H_SYNC_LEN time elapsed

		BACK_PORCH:
		n_h_state = (pixel_index + 16'd1 == `H_BACK_PORCH_LEN) ? ACTIVE : h_state; // switch to ACTIVE state if H_BACK_PORCH_LEN time elapsed

		ACTIVE:
		n_h_state = (pixel_index + 16'd1 == `WIDTH) ? FRONT_PORCH : h_state; // switch to FRONT_PORCH state if WIDTH time elapsed

		default:
		n_h_state = FRONT_PORCH; // go to FRONT_PORCH state by default
	endcase

	if(n_h_state == h_state) n_pixel_index = pixel_index + 16'd1; // if not switching states, increment the pixel index
	else n_pixel_index = 'd0; // otherwise reset it to zero

	// assert the horizontal sync pulse signal if in SYNC state
	if(h_state == SYNC) h_sync = 'b0;
	else h_sync = 'b1;

	if(n_h_state == SYNC && h_state != SYNC) begin // if moving on to the next line...
		case(v_state) // vertical FSM
			FRONT_PORCH:
			n_v_state = (line_index + 16'd1 == `V_FRONT_PORCH_LEN) ? SYNC : v_state; // switch to SYNC state if V_FRONT_PORCH_LEN time elapsed

			SYNC:
			n_v_state = (line_index + 16'd1 == `V_SYNC_LEN) ? BACK_PORCH : v_state; // switch to BACK_PORCH state if V_SYNC_LEN time elapsed

			BACK_PORCH:
			n_v_state = (line_index + 16'd1 == `V_BACK_PORCH_LEN) ? ACTIVE : v_state; // switch to ACTIVE state if V_BACK_PORCH_LEN time elapsed

			ACTIVE:
			n_v_state = (line_index + 16'd1 == `HEIGHT) ? FRONT_PORCH : v_state; // switch to FRONT_PORCH state if HEIGHT time elapsed

			default:
			n_v_state = FRONT_PORCH; // go to FRONT_PORCH state by default
		endcase

		if(n_v_state == v_state) n_line_index = line_index + 16'd1; // if not switching states, increment the line index
		else n_line_index = 'd0; // otherwise reset it to zero
	end
	else begin // if not switching lines...
		// maintain the vertical state and line index
		n_v_state = v_state;
		n_line_index = line_index;
	end

	// assert the vertical sync pulse signal if in SYNC state
	if(v_state == SYNC) v_sync = 'b0;
	else v_sync = 'b1;

	// assert the blanking signal if the FSMs are not both in the ACTIVE state
	if(v_state == ACTIVE && h_state == ACTIVE) blanking = 'b0;
	else blanking = 'b1;
end
endmodule
