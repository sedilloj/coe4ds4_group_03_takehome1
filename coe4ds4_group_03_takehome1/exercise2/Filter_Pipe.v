// Copyright by Adam Kinsman, Henry Ko and Nicola Nicolici
// Developed for the Embedded Systems course (COE4DS4)
// Department of Electrical and Computer Engineering
// McMaster University
// Ontario, Canada

`timescale 1ns/100ps
`default_nettype none

// This module implements the image filter pipe
module Filter_Pipe (
	input logic Clock,
	input logic Clock_en,
	input logic Resetn,

	input logic Enable,
	input logic [31:0] Filter_config,

	input logic [10:0] H_Count,
	input logic [9:0] V_Count,

	output logic oRead_in_en,
	input logic [7:0] R_in,
	input logic [7:0] G_in,
	input logic [7:0] B_in,
	
	input logic iRead_out_en,
	output logic [7:0] R_out,
	output logic [7:0] G_out,
	output logic [7:0] B_out
);

always_ff @(posedge Clock or negedge Resetn) begin
	if (~Resetn) oRead_in_en <= 1'b0;
	else if (~Enable) oRead_in_en <= 1'b0;
	else oRead_in_en <= (
		(H_Count > (11'd216 - 11'd2)) &&
		(H_Count < (11'd216 - 11'd2 + 11'd640 + 11'd1)) &&
		(V_Count > (10'd35 - 10'd1 - 10'd1 - 10'd1)) &&
		(V_Count < (10'd35 - 10'd1 + 10'd480 + 10'd1 - 10'd1 - 10'd1))
	) ? ~Clock_en : 1'b0;
end

logic [7:0] Red_rddata_0b, Green_rddata_0b, Blue_rddata_0b;
logic [7:0] Red_wrdata_1a, Green_wrdata_1a, Blue_wrdata_1a;

logic DP_wren_0a;
logic [9:0] DP_addr_0a, DP_addr_0b;

always_ff @(posedge Clock or negedge Resetn) begin
	if (~Resetn) begin
		DP_wren_0a <= 1'b0;
		DP_addr_0a <= 10'd0;
	end else if (~Enable) begin
		DP_wren_0a <= 1'b0;
		DP_addr_0a <= 10'd0;		
	end else if (Clock_en) begin
		DP_wren_0a <= oRead_in_en;
		if (DP_wren_0a) DP_addr_0a <= DP_addr_0a + 10'd1;
	end
end

Filter_RAM Filter_RAM_R0 (
	.address_a(DP_addr_0a),
	.address_b(DP_addr_0b),
	.clock(Clock),
	.data_a(R_in),
	.data_b(8'h00),
	.wren_a(DP_wren_0a),
	.wren_b(1'b0),
	.q_a(),
	.q_b(Red_rddata_0b)
);

Filter_RAM Filter_RAM_G0 (
	.address_a(DP_addr_0a),
	.address_b(DP_addr_0b),
	.clock(Clock),
	.data_a(G_in),
	.data_b(8'h00),
	.wren_a(DP_wren_0a),
	.wren_b(1'b0),
	.q_a(),
	.q_b(Green_rddata_0b)
);

Filter_RAM Filter_RAM_B0 (
	.address_a(DP_addr_0a),
	.address_b(DP_addr_0b),
	.clock(Clock),
	.data_a(B_in),
	.data_b(8'h00),
	.wren_a(DP_wren_0a),
	.wren_b(1'b0),
	.q_a(),
	.q_b(Blue_rddata_0b)
);

logic Read_0_en;
logic [9:0] DP_addr_0b_reg;

logic DP_wren_1a;
logic [9:0] DP_addr_1a, DP_addr_1b;

logic [7:0] Y_calc, Red_calc, Green_calc, Blue_calc;
logic [21:0] Y_calc_long;

assign Y_calc_long = 
	(22'd1052 * Red_rddata_0b) + 
	(22'd2064 * Green_rddata_0b) + 
	(22'd401 * Blue_rddata_0b);
assign Y_calc = Y_calc_long[19:12];

logic [7:0] Y_m1, Y_0, Y_p1;
logic [4:0] filter_en;

logic [11:0] Filter_calc_1;
logic [7:0] Filter_calc_2;

logic [9:0] Filter_calc_3_pre;
logic [7:0] Filter_calc_3;

//exericse2 instantiation
logic [7:0] low,high;
logic [9:0] sum;
logic [7:0] Y_m2, Y_p2;
logic [6:0] filterThres;
logic [9:0] Filter_calc_4_pre;
logic [7:0] Filter_calc_4;
logic [9:0] Filter_calc_5_pre;
logic [7:0] Filter_calc_5;

assign Filter_calc_1 = {4'h0, Y_0} + {4'h0, Y_p1};
assign Filter_calc_2 = Filter_calc_1[8:1];

//our filter
assign Filter_calc_3_pre = (Y_m1 + {Y_0,1'b0} + Y_p1);
assign Filter_calc_3 = Filter_calc_3_pre[9:2];

//exercise2 filter 6
assign Filter_calc_4_pre = $signed({Y_p1,1'b0}+{Y_p2,1'b0}-{Y_m2,1'b0}-{Y_m1,1'b0});

// exercise2; filter 6
always_comb begin
	if (Filter_config[3]) begin
		if ($signed(Filter_calc_4_pre) < $signed(-8'd128)) begin
			Filter_calc_4 = 8'b0;
		end else if ($signed(Filter_calc_4_pre) > 8'd127) begin
			Filter_calc_4 = 8'd255;
		end else begin
			Filter_calc_4 = Filter_calc_4_pre[7:0];
		end
	end else begin
		if ($signed(Filter_calc_4_pre) < $signed(-8'd64)) begin
			Filter_calc_4 = 8'b0;
		end else if ($signed(Filter_calc_4_pre) > 8'd63) begin
			Filter_calc_4 = 8'd255;
		end else begin
			Filter_calc_4 = Filter_calc_4_pre[7:0];
		end
	end
end

//exercise2 filter 7 - median
always_comb begin
	if (Y_m1 < Y_m2)
		low = Y_m1;
	else
		low = Y_m2;
	if (low > Y_p1)
		low = Y_p1;
	if (low > Y_p2)
		low = Y_p2;
	
	if (Y_m1 > Y_m2)
		high = Y_m1;
	else
		high = Y_m2;
	if (high < Y_p1)
		high = Y_p1;
	if (high < Y_p2)
		high = Y_p2;
		
	sum = Y_m1 + Y_m2 + Y_p1 + Y_p2;
	
	Filter_calc_5_pre = sum - low - high;
	Filter_calc_5 = Filter_calc_5_pre[8:1];
		
end

always_ff @(posedge Clock or negedge Resetn) begin
	if (~Resetn) begin
		DP_addr_0b <= 10'h000;
		filter_en <= 5'b00000;
		DP_wren_1a <= 1'b0;
	end else if (~Enable) begin
		DP_addr_0b <= 10'h000;
		filter_en <= 5'b00000;
		DP_wren_1a <= 1'b0;
	end else if (Clock_en) begin
		DP_addr_0b <= DP_addr_0a;
		filter_en <= {filter_en[3:0],DP_wren_0a};
		if (Filter_config[2:0] == 3'd4)
			DP_wren_1a <= filter_en[3];
		else if (Filter_config[2:0] == 3'd5)
			DP_wren_1a <= filter_en[4];
		else DP_wren_1a <= filter_en[1];
	end
end

always_ff @(posedge Clock or negedge Resetn) begin
	if (~Resetn) begin		
		Y_m1 <= 8'd0;
		Y_0 <= 8'd0;
		Y_p1 <= 8'd0;
		
		//exercise2
		Y_p2 <= 8'd0;
		Y_m2 <= 8'd0;
	end else if (Clock_en) begin
		Y_0 <= Y_p1;
		Y_p1 <= Y_p2;

		// for lead-in corner case
		if (filter_en[3] & ~filter_en[4] && Filter_config[2:0] >= 3'd6) begin
			Y_m1 <= Y_p1;
			Y_m2 <= Y_p1;
		end else if (filter_en[2] & ~filter_en[3]) begin
			Y_m1 <= Y_p1;
		end else begin
			Y_m1 <= Y_0;
			Y_m2 <= Y_m1;
		end
		
		// for take-down corner case
		if (filter_en[1])
			Y_p2 <= Y_calc;
	end
end

always_ff @(posedge Clock or negedge Resetn) begin
	if (~Resetn) begin		
		Red_wrdata_1a <= 8'h00;
		Green_wrdata_1a <= 8'h00;
		Blue_wrdata_1a <= 8'h00;
	end else if (Clock_en) begin
		case (Filter_config[2:0])
			3'd0 : begin
				Red_wrdata_1a <= Red_rddata_0b;
				Green_wrdata_1a <= Green_rddata_0b;
				Blue_wrdata_1a <= Blue_rddata_0b;
			end
			3'd1 : begin
				Red_wrdata_1a <= ~Red_rddata_0b;
				Green_wrdata_1a <= ~Green_rddata_0b;
				Blue_wrdata_1a <= ~Blue_rddata_0b;
			end
			3'd2 : begin
				Red_wrdata_1a <= Y_calc;
				Green_wrdata_1a <= Y_calc;
				Blue_wrdata_1a <= Y_calc;
			end
			3'd3 : begin
				Red_wrdata_1a <= ~Y_calc;
				Green_wrdata_1a <= ~Y_calc;
				Blue_wrdata_1a <= ~Y_calc;
			end
			3'd4 : begin
				Red_wrdata_1a <= Filter_calc_2;
				Green_wrdata_1a <= Filter_calc_2;
				Blue_wrdata_1a <= Filter_calc_2;
			end
			3'd5 : begin
				Red_wrdata_1a <= Filter_calc_3;
				Green_wrdata_1a <= Filter_calc_3;
				Blue_wrdata_1a <= Filter_calc_3;
			end
			//exercise2
			3'd6 : begin
				Red_wrdata_1a <= Filter_calc_4;
				Green_wrdata_1a <= Filter_calc_4;
				Blue_wrdata_1a <= Filter_calc_4;
			end
			//exercise2
			3'd7 : begin
				Red_wrdata_1a <= Filter_calc_5;
				Green_wrdata_1a <= Filter_calc_5;
				Blue_wrdata_1a <= Filter_calc_5;
			end
		endcase
	end
end

always_ff @(posedge Clock or negedge Resetn) begin
	if (~Resetn) DP_addr_1a <= 10'd0;
	else if (~Enable) DP_addr_1a <= 10'd0;
	else if (Clock_en & DP_wren_1a) 
		DP_addr_1a <= DP_addr_1a + 10'd1;
end

Filter_RAM Filter_RAM_R1 (
	.address_a(DP_addr_1a),
	.address_b(DP_addr_1b),
	.clock(Clock),
	.data_a(Red_wrdata_1a),
	.data_b(8'h00),
	.wren_a(DP_wren_1a),
	.wren_b(1'b0),
	.q_a(),
	.q_b(R_out)
);

Filter_RAM Filter_RAM_G1 (
	.address_a(DP_addr_1a),
	.address_b(DP_addr_1b),
	.clock(Clock),
	.data_a(Green_wrdata_1a),
	.data_b(8'h00),
	.wren_a(DP_wren_1a),
	.wren_b(1'b0),
	.q_a(),
	.q_b(G_out)
);

Filter_RAM Filter_RAM_B1 (
	.address_a(DP_addr_1a),
	.address_b(DP_addr_1b),
	.clock(Clock),
	.data_a(Blue_wrdata_1a),
	.data_b(8'h00),
	.wren_a(DP_wren_1a),
	.wren_b(1'b0),
	.q_a(),
	.q_b(B_out)
);

logic [9:0] DP_addr_1b_reg;
assign DP_addr_1b = DP_addr_1b_reg + ((iRead_out_en) ? 10'd1 : 10'd0);

always_ff @(posedge Clock or negedge Resetn) begin
	if (~Resetn) DP_addr_1b_reg <= 10'h3FF;
	else if (~Enable) DP_addr_1b_reg <= 10'h3FF;
	else if (Clock_en)
		DP_addr_1b_reg <= DP_addr_1b;
end

endmodule
