// Copyright by Adam Kinsman, Henry Ko and Nicola Nicolici
// Developed for the Embedded Systems course (COE4DS4)
// Department of Electrical and Computer Engineering
// McMaster University
// Ontario, Canada

`timescale 1ns/100ps
`default_nettype none

// This is the top module
// It interfaces to the LCD display and touch panel
module exercise1 (
	/////// board clocks                      ////////////
	input logic CLOCK_50_I,                   // 50 MHz clock
	
	/////// pushbuttons/switches              ////////////
	input logic[3:0] PUSH_BUTTON_I,           // pushbuttons
	input logic[17:0] SWITCH_I,               // toggle switches
	
	/////// 7 segment displays/LEDs           ////////////
	output logic[6:0] SEVEN_SEGMENT_N_O[7:0], // 8 seven segment displays
	output logic[8:0] LED_GREEN_O,            // 9 green LEDs
	output logic[17:0] LED_RED_O,             // 18 red LEDs
	
	/////// GPIO connections                  ////////////
	inout wire[35:0] GPIO_0                   // GPIO Connection 0 (LTM)
);

// Signals for LCD Touch Module (LTM)
// LCD display interface
logic 	[7:0]	LTM_R, LTM_G, LTM_B;
logic 			LTM_HD, LTM_VD;
logic 			LTM_NCLK, LTM_DEN, LTM_GRST;

// LCD configuration interface
wire 			LTM_SDA;
logic 			LTM_SCLK, LTM_SCEN;

// LCD touch panel interface
logic 			TP_DCLK, TP_CS, TP_DIN, TP_DOUT;
logic 			TP_PENIRQ_N, TP_BUSY;

// Internal signals
logic 			Clock, Resetn;
logic 	[2:0] 	Top_state;

// For LCD display / touch screen
logic 			LCD_TPn_sel, LCD_TPn_sclk;
logic 			LCD_config_start, LCD_config_done;
logic 			LCD_enable, TP_enable;
logic 			TP_touch_en, TP_coord_en;
logic 	[11:0]	TP_X_coord, TP_Y_coord;

logic 	[9:0] 	Colourbar_X, Colourbar_Y;
logic 	[7:0]	Colourbar_Red, Colourbar_Green, Colourbar_Blue;

logic 	[4:0] 	TP_position[7:0];

// new variables
logic 	[2:0] 	colMost, countGreatest, colGrid[3:0], colCounter[7:0], colCountInc, inc[7:0];
logic 	[3:0]		i, j;
logic 				enChange;
logic 	[25:0] 	clkCounter;
logic		[13:0]	msCounter;
logic 	[4:0] 	activeQuad;

// end of experiment variables

assign Clock = CLOCK_50_I;
assign Resetn = SWITCH_I[17];

assign LCD_TPn_sclk = (LCD_TPn_sel) ? LTM_SCLK : TP_DCLK;
assign LTM_SCEN = (LCD_TPn_sel) ? 1'b0 : ~TP_CS;
assign LTM_GRST = Resetn;

// Connections to GPIO for LTM
assign TP_PENIRQ_N   = GPIO_0[0];
assign TP_DOUT       = GPIO_0[1];
assign TP_BUSY       = GPIO_0[2];
assign GPIO_0[3]	 = TP_DIN;

assign GPIO_0[4]	 = LCD_TPn_sclk;

assign GPIO_0[35]    = LTM_SDA;
assign GPIO_0[34]    = LTM_SCEN;
assign GPIO_0[33]    = LTM_GRST;

assign GPIO_0[9]	 = LTM_NCLK;
assign GPIO_0[10]    = LTM_DEN;
assign GPIO_0[11]    = LTM_HD;
assign GPIO_0[12]    = LTM_VD;

assign GPIO_0[5]     = LTM_B[3];
assign GPIO_0[6]     = LTM_B[2];
assign GPIO_0[7]     = LTM_B[1];
assign GPIO_0[8]     = LTM_B[0];
assign GPIO_0[16:13] = LTM_B[7:4];
assign GPIO_0[24:17] = LTM_G[7:0];
assign GPIO_0[32:25] = LTM_R[7:0];

// Top state machine for controlling resets
always_ff @(posedge Clock or negedge Resetn) begin
	if (~Resetn) begin
		Top_state <= 3'h0;
		TP_enable <= 1'b0;
		LCD_enable <= 1'b0;
		LCD_config_start <= 1'b0;
		LCD_TPn_sel <= 1'b1;
	end else begin
		case (Top_state)
			3'h0 : begin
				LCD_config_start <= 1'b1;
				LCD_TPn_sel <= 1'b1;
				Top_state <= 3'h1;
			end			
			3'h1 : begin
				LCD_config_start <= 1'b0;
				if (LCD_config_done & ~LCD_config_start) begin
					TP_enable <= 1'b1;
					LCD_enable <= 1'b1;
					LCD_TPn_sel <= 1'b0;
					Top_state <= 3'h2;
				end
			end			
			3'h2 : begin
				Top_state <= 3'h2;
			end
		endcase
	end
end				

// LCD Configuration
LCD_Config_Controller LCD_Config_unit(
	.Clock(Clock),
	.Resetn(Resetn),
	.Start(LCD_config_start),
	.Done(LCD_config_done),
	.LCD_I2C_sclk(LTM_SCLK),
 	.LCD_I2C_sdat(LTM_SDA),
	.LCD_I2C_scen()
);

// LCD Image
LCD_Data_Controller LCD_Data_unit (
	.Clock(Clock),
	.oClock_en(),
	.Resetn(Resetn),
	.Enable(LCD_enable),
	.iRed(Colourbar_Red),
	.iGreen(Colourbar_Green),
	.iBlue(Colourbar_Blue),
	.oCoord_X(Colourbar_X),
	.oCoord_Y(Colourbar_Y),
	.H_Count(), // not used in this experiment
	.V_Count(), // not used in this experiment
	.LTM_NCLK(LTM_NCLK),
	.LTM_HD(LTM_HD),
	.LTM_VD(LTM_VD),
	.LTM_DEN(LTM_DEN),
	.LTM_R(LTM_R),
	.LTM_G(LTM_G),
	.LTM_B(LTM_B)
);

// State machine for generating the colour bars
always_ff @(posedge Clock or negedge Resetn) begin
	if (~Resetn) begin
		Colourbar_Red <= 8'h00; 
		Colourbar_Green <= 8'h00;
		Colourbar_Blue <= 8'h00;
	end else begin
		if (Colourbar_X < 10'd400 && Colourbar_Y < 10'd240) begin
			Colourbar_Red <= {8{colGrid[0][2]}};
			Colourbar_Green <= {8{colGrid[0][1]}};
			Colourbar_Blue <= {8{colGrid[0][0]}};
		end else if (Colourbar_X < 10'd400 && Colourbar_Y > 10'd239) begin
			Colourbar_Red <= {8{colGrid[2][2]}};
			Colourbar_Green <= {8{colGrid[2][1]}};
			Colourbar_Blue <= {8{colGrid[2][0]}};
		end else if (Colourbar_X > 10'd399 && Colourbar_Y < 10'd240) begin
			Colourbar_Red <= {8{colGrid[1][2]}};
			Colourbar_Green <= {8{colGrid[1][1]}};
			Colourbar_Blue <= {8{colGrid[1][0]}};
		end else if (Colourbar_X > 10'd399 && Colourbar_Y > 10'd239) begin
			Colourbar_Red <= {8{colGrid[3][2]}};
			Colourbar_Green <= {8{colGrid[3][1]}};
			Colourbar_Blue <= {8{colGrid[3][0]}};
		end
	end
end

// Controller for the TP on the LTM
Touch_Panel_Controller Touch_Panel_unit(
	.Clock_50MHz(Clock),
	.Resetn(Resetn),
	.Enable(~LTM_VD),	
	.Touch_En(TP_touch_en),
	.Coord_En(TP_coord_en),
	.X_Coord(TP_X_coord),
	.Y_Coord(TP_Y_coord),
	.TP_PENIRQ_N_I(TP_PENIRQ_N),
	.TP_BUSY_I(TP_BUSY),
	.TP_SCLK_O(TP_DCLK),
	.TP_MOSI_O(TP_DIN),
	.TP_MISO_I(TP_DOUT),
	.TP_SS_N_O(TP_CS)
);

// State machine for controlling colours
always_ff @(posedge Clock or negedge Resetn) begin
	if (~Resetn) begin
		colGrid[0] <= 3'h000;
		colGrid[1] <= 3'b111;
		colGrid[2] <= 3'b111;
		colGrid[3] <= 3'h000;
		
		msCounter <= 1'b0;
		
		colCounter[7] <= 6'b0;
		colCounter[6] <= 6'b0;
		colCounter[5] <= 6'b0;
		colCounter[4] <= 6'b0;
		colCounter[3] <= 6'b0;
		colCounter[2] <= 6'b0;
		colCounter[1] <= 6'b0;
		colCounter[0] <= 6'b0;
		colMost <= 1'b0;
		countGreatest <= 1'b0;
	end else begin
		enChange <= 1'b0;
		clkCounter <= 1'b0;
		
		if (TP_touch_en) begin
			clkCounter <= clkCounter + 26'd1;
			if (clkCounter == 16'd49_999) begin
				// Counter
				if (activeQuad == TP_position[7][4:0]) begin
					msCounter <= msCounter + 1'b1;
					clkCounter <= 26'd0;
					
					if (msCounter == 10'd999) begin
						enChange <= 1'b1;
						msCounter <= 1'b0;
					end
				end else begin
					msCounter <= 1'b0;
				end
				
				// BCD Counter
				TP_position[0][3:0] <= TP_position[0][3:0] + 1;
				if (TP_position[0][3:0] == 9) begin
					TP_position[1][3:0] <= TP_position[1][3:0] + 1;
					TP_position[0][3:0] <= 1'b0;
				end 
				if (TP_position[1][3:0] == 9) begin
					TP_position[2][3:0] <= TP_position[2][3:0] + 1;
					TP_position[1][3:0] <= 1'b0;
				end 
				if (TP_position[2][3:0] == 9) begin
					TP_position[3][3:0] <= TP_position[3][3:0] + 1;
					TP_position[2][3:0] <= 1'b0;
				end
			end
			
		end else begin
			// Disable/Reset BCD Counters
			TP_position[3][3:0] <= 1'b0;
			TP_position[2][3:0] <= 1'b0;
			TP_position[1][3:0] <= 1'b0;
			
			msCounter <= 1'b0;
			
			// Most Present Colour
			colCounter[7] = 6'b0;
			colCounter[6] = 6'b0;
			colCounter[5] = 6'b0;
			colCounter[4] = 6'b0;
			colCounter[3] = 6'b0;
			colCounter[2] = 6'b0;
			colCounter[1] = 6'b0;
			colCounter[0] = 6'b0;
			countGreatest = 0;
			
			for (i=0; i<3'd4; i=i+1) begin
				colCounter[colGrid[i]] = colCounter[colGrid[i]] + 1'b1;
			end
			
			for (i=0; i<4'd8; i=i+1) begin
				if (colCounter[i] >= countGreatest) begin
					countGreatest = colCounter[i];
					colMost <= i;
				end
			end
			
			TP_position[0][3:0] <= colMost;
		end
		
		if (enChange) begin
			case ({TP_X_coord[11],TP_Y_coord[11]})
				2'b00: colGrid[0] <= colGrid[0] + 1'b1;
				2'b01: colGrid[2] <= colGrid[2] - 1'b1;
				2'b10: colGrid[1] <= colGrid[1] - 1'b1;
				2'b11: colGrid[3] <= colGrid[3] + 1'b1;
			endcase
		end
	end
end

// State machine for capturing the touch panel coordinates
// and displaying them on the seven segment displays
always_ff @(posedge Clock or negedge Resetn) begin
	if (~Resetn) begin
		TP_position[0][4] <= 1'b1;
		TP_position[1][4] <= 1'b0;
		TP_position[2][4] <= 1'b0;
		TP_position[3][4] <= 1'b0;
		TP_position[4][4] <= 1'b0;
		TP_position[5][4] <= 1'b0;
		TP_position[6][4] <= 1'b0;
		TP_position[7][4] <= 1'b0;
	end else begin
		activeQuad <= TP_position[7][4:0];
		
		if (~TP_touch_en) begin
			TP_position[7][4] <= 1'b0;
			TP_position[3][4] <= 1'b0;
			TP_position[2][4] <= 1'b0;
			TP_position[1][4] <= 1'b0;
		end else begin
			if (TP_coord_en) begin
				// Timer
				TP_position[3][4] <= 1'b1;
				TP_position[2][4] <= 1'b1;
				TP_position[1][4] <= 1'b1;
				TP_position[0][4] <= 1'b1;
				
				// Active Quadrant
				if (~TP_X_coord[11] && ~TP_Y_coord[11]) begin
					TP_position[7][4:0] <= 5'b10000;
				end else if (~TP_X_coord[11] && TP_Y_coord[11]) begin
					TP_position[7][4:0] <= 5'b10010;
				end else if (TP_X_coord[11] && ~TP_Y_coord[11]) begin
					TP_position[7][4:0] <= 5'b10001;
				end else if (TP_X_coord[11] && TP_Y_coord[11]) begin
					TP_position[7][4:0] <= 5'b10011;
				end
			end
		end
	end
end

// Seven segment displays
seven_seg_displays display_unit (
	.hex_values(TP_position),
	.SEVEN_SEGMENT_N_O(SEVEN_SEGMENT_N_O)
);

endmodule
