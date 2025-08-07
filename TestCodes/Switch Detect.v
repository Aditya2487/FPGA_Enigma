////////// Switch Detect and Display Modularised

module Testing3(sys_clk,rst,input_sw, led1, shift_pulse,latch_clk,sdata);
	input sys_clk,rst;
	input [39:0] input_sw;
	output  reg [2:0] led1= 3'b000 ;
	output shift_pulse,latch_clk,sdata;
	
	
	wire [39:0] keyOutput;
		
	shifter40bit charDisplayUnit(sys_clk,rst,keyOutput,shift_pulse,latch_clk,sdata);
	switchDetect switchScanner(sys_clk,rst,input_sw,keyOutput);

endmodule
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

module switchDetect(
	input sys_clk,rst,
	input [39:0] input_sw,
	output [39:0] keyOutput);

	reg [2:0] state = IDLE;
	reg [25:0] delayCounter = 0;
	reg [5:0] active_switch = 0;
	reg [39:0] scrambledOP = 40'd0;
	
	unscrambler unsc1(scrambledOP,keyOutput);
	
	parameter IDLE =			3'b000;
	parameter TRIG = 			3'b001;
	parameter LED0 =			3'b010;
	parameter LED1 =			3'b011;
	parameter LED2 = 			3'b100;
	parameter LED3 = 			3'b101;
	parameter CHECK_HIGH = 	3'b111;
	
always @(posedge sys_clk) 
begin
	delayCounter <= delayCounter + 1 ;
    if (rst) begin
            state <= IDLE;
            delayCounter <= 0;
            scrambledOP <= 40'd0;
             active_switch <= 0;
	 end
	 else
	 begin
		case (state)
			IDLE : 
				begin
					if(!input_sw[0]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 0; end
					else if(!input_sw[1]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 1; end
					else if(!input_sw[2]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 2; end
					else if(!input_sw[3]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 3; end
						else if(!input_sw[4]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 4; end
						else if(!input_sw[5]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 5; end
						else if(!input_sw[6]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 6; end
						else if(!input_sw[7]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 7; end
						else if(!input_sw[8]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 8; end
						else if(!input_sw[9]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 9; end
						else if(!input_sw[10]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 10; end
						else if(!input_sw[11]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 11; end
						else if(!input_sw[12]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 12; end
						else if(!input_sw[13]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 13; end
						else if(!input_sw[14]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 14; end
						else if(!input_sw[15]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 15; end
						else if(!input_sw[16]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 16; end
						else if(!input_sw[17]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 17; end
						else if(!input_sw[18]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 18; end
						else if(!input_sw[19]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 19; end
						else if(!input_sw[20]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 20; end
						else if(!input_sw[21]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 21; end
						else if(!input_sw[22]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 22; end
						else if(!input_sw[23]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 23; end
						else if(!input_sw[24]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 24; end
						else if(!input_sw[25]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 25; end
						else if(!input_sw[26]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 26; end
						else if(!input_sw[27]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 27; end
						else if(!input_sw[28]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 28; end
						else if(!input_sw[29]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 29; end
						else if(!input_sw[30]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 30; end
						else if(!input_sw[31]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 31; end
						else if(!input_sw[32]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 32; end
						else if(!input_sw[33]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 33; end
						else if(!input_sw[34]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 34; end
						else if(!input_sw[35]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 35; end
						else if(!input_sw[36]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 36; end
						else if(!input_sw[37]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 37; end
						else if(!input_sw[38]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 38; end
						else if(!input_sw[39]) begin delayCounter <=0 ; state <= TRIG ; active_switch <= 39; end
	
					else begin state <= IDLE; end
				end
			TRIG :
				begin
					if( delayCounter == 5_000_000 ) 
					begin
						delayCounter <= 0;
						if( !input_sw[active_switch] ) begin
							state <= LED0;
						end
						else
						begin
							state <= IDLE;	
							active_switch <= 0 ;
						end
					end
				end
			LED0 :
				begin
					if( delayCounter == 6_000_000 ) 
					begin
						delayCounter <= 0;
						state <= LED1;
						scrambledOP[active_switch] <= ~(scrambledOP[active_switch]) ;
					end
				end
			LED1 :
				begin
					if( delayCounter == 6_000_000 ) 
					begin
						delayCounter <= 0;
						state <= LED2;
						scrambledOP[active_switch] <= ~(scrambledOP[active_switch]) ;
					end
				end
			LED2 :
				begin
					if( delayCounter == 6_000_000 ) 
					begin
						delayCounter <= 0;
						state <= LED3;
						scrambledOP[active_switch] <= ~(scrambledOP[active_switch]) ;
					end
				end
			LED3 :
				begin
					if( delayCounter == 6_000_000 ) 
					begin
						delayCounter <= 0;
						state <= CHECK_HIGH;
						scrambledOP[active_switch] <= ~(scrambledOP[active_switch]) ;
					end
				end	
			CHECK_HIGH :
				begin
					if (input_sw[active_switch]) begin
						state <= IDLE;
						active_switch <=0;
					end
					else begin
						state <= CHECK_HIGH;
					end	
				end
		endcase
	 end
end

endmodule

///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
module unscrambler (input [39:0] scrambledOP, output [39:0] keyOutput );

assign keyOutput[0] = scrambledOP[38];
assign keyOutput[1] = scrambledOP[18];
assign keyOutput[2] = scrambledOP[37];
assign keyOutput[3] = scrambledOP[17];
assign keyOutput[4] = scrambledOP[36];
assign keyOutput[5] = scrambledOP[22];
assign keyOutput[6] = scrambledOP[6];
assign keyOutput[7] = scrambledOP[21];
assign keyOutput[8] = scrambledOP[5];
assign keyOutput[9] = scrambledOP[20];
assign keyOutput[10] = scrambledOP[16];
assign keyOutput[11] = scrambledOP[35];
assign keyOutput[12] = scrambledOP[0];
assign keyOutput[13] = scrambledOP[34];
assign keyOutput[14] = scrambledOP[1];
assign keyOutput[15] = scrambledOP[9];
assign keyOutput[16] = scrambledOP[24];
assign keyOutput[17] = scrambledOP[8];
assign keyOutput[18] = scrambledOP[23];
assign keyOutput[19] = scrambledOP[7];
assign keyOutput[20] = scrambledOP[33];
assign keyOutput[21] = scrambledOP[2];
assign keyOutput[22] = scrambledOP[32];
assign keyOutput[23] = scrambledOP[3];
assign keyOutput[24] = scrambledOP[31];
assign keyOutput[25] = scrambledOP[27];
assign keyOutput[26] = scrambledOP[11];
assign keyOutput[27] = scrambledOP[26];
assign keyOutput[28] = scrambledOP[10];
assign keyOutput[29] = scrambledOP[25];
assign keyOutput[30] = scrambledOP[19];
assign keyOutput[31] = scrambledOP[4];
assign keyOutput[32] = scrambledOP[30];
assign keyOutput[33] = scrambledOP[15];
assign keyOutput[34] = scrambledOP[29];
assign keyOutput[35] = scrambledOP[14];
assign keyOutput[36] = scrambledOP[28];
assign keyOutput[37] = scrambledOP[13];
assign keyOutput[38] = scrambledOP[39];
assign keyOutput[39] = scrambledOP[12];

endmodule
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////




module shifter40bit(
	input sys_clk,rst,
	input [39:0] data_input,
	output reg shift_pulse,latch_clk,sdata );

parameter RESET = 2'b00;
parameter IDLE = 2'b01;
parameter SHIFT = 2'b10;
parameter LATCH = 2'b11;

reg [39:0] dataTOshift = (40'd0);
reg [5:0] index = 6'b000000;
reg [1:0] state = RESET; 
reg [4:0] counter = 5'd0;


always @(posedge sys_clk) begin
	if(dataTOshift != data_input)begin
		dataTOshift <= data_input;
		state <= RESET;
		index <= 0;		
		counter <= 0;
		latch_clk <= 0;
		shift_pulse <=0;
		sdata <=0;
	end
	if (rst) begin
		state <= RESET;
		index <= 0;		
		counter <= 0;
		latch_clk <= 0;
		shift_pulse <=0;
		sdata <=0;
		end
	else begin
		counter  <= counter + 1 ;
		if( counter == 4'd20 ) begin
			counter <= 0 ;
			case (state) 
				
				RESET : 
					begin
					index <= 0;		
					latch_clk <= 0;
					shift_pulse <=0;
					sdata <=0;
					state <= IDLE ;
					dataTOshift <= data_input;
					end
				
				IDLE :
					begin
					latch_clk <= 0;
					shift_pulse <= 0;
					sdata <= dataTOshift[index];
					state <= SHIFT;
					end
				SHIFT :
					begin
					latch_clk <= 0;
					shift_pulse <= 1;
					if(index == 6'd39)
						begin
						state <= LATCH;
						end
					else
						begin
						state <= IDLE;
						index <= index + 1 ;
						end
					end
				LATCH:
					begin
					latch_clk <= 1;
					shift_pulse <= 0;
					state <= IDLE;
					index <= 0;
					end
			endcase
		
		end
	end
	


end 




endmodule
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
