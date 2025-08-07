// Combined Module 2

module EnigmaMain(sys_clk,rst,encoderIP1,encoderIP2,encoderIP3,input_sw, led1, shift_pulse,latch_clk,sdata,shift_pulse2,latch_clk2,sdata2,buzzer);
	input sys_clk,rst;
	input encoderIP1,encoderIP2,encoderIP3;
	input [39:0] input_sw;
	output  reg [2:0] led1= 3'b000 ;
	output shift_pulse,latch_clk,sdata;
	output shift_pulse2,latch_clk2,sdata2;
	output buzzer;
	
	
	wire [23:0] decodedop;
	wire [11:0] encryptKey;
	wire [39:0] keyOutput;
	wire [39:0] encrptedData;
	wire keyTrigger;
	
	assign keyTrigger = |keyOutput;
	assign buzzer = keyTrigger;
	
	switchDetect switchScanner(sys_clk,rst,input_sw,keyOutput);
	
	EncrypterModule encMain(keyOutput,encryptKey[11:8],encryptKey[7:4],encryptKey[3:0],encrptedData);
	
	shifter40bit charDisplayUnit(sys_clk,rst,encrptedData,shift_pulse,latch_clk,sdata);
	
	encoderUnit Encoder1(sys_clk,encoderIP1,encoderIP2,encoderIP3,keyTrigger,encryptKey);
	displayDriver diplayencoderTest(encryptKey,decodedop);
	shifter24bit charDisplayUnitTest(sys_clk,rst,decodedop,shift_pulse2,latch_clk2,sdata2);

	
	
endmodule

module encoderUnit(sys_clk,encoderPulse1,encoderPulse2,encoderPulse3,keyTrigger,testData);
	input sys_clk,encoderPulse1,encoderPulse2,encoderPulse3,keyTrigger;
	output reg [11:0] testData = 12'd0;
	
	reg [30:0] delayCounter1 = 0;
	reg prevEncoderPulse1 = 1'd0;
	
	reg [30:0] delayCounter2 = 0;
	reg prevEncoderPulse2 = 1'd0;
	
	reg [30:0] delayCounter3 = 0;
	reg prevEncoderPulse3 = 1'd0;
	
	reg [30:0] delayCounter4 = 0;

always @(negedge sys_clk)
begin
		delayCounter1 <= delayCounter1 + 1;
		delayCounter2 <= delayCounter2 + 1;
		delayCounter3 <= delayCounter3 + 1;
		delayCounter4 <= delayCounter4 + 1;
		if(keyTrigger)
		begin
			if(delayCounter4 > 30'd12_500_000) 
			begin
				delayCounter4 <= 0;
				testData <= testData + 1 ;
			end
		end
		else 
			begin
		
				if(prevEncoderPulse1 == 1 && encoderPulse1 == 0)
				begin
					if(delayCounter1 > 31'd1_950_000)
					begin
					testData[11:8] <= testData[11:8] + 1 ;
					delayCounter1 <= 0;
					end
				end
				
				if(prevEncoderPulse2 == 1 && encoderPulse2 == 0)
				begin
					if(delayCounter2 > 31'd1_950_000)
					begin
					testData[7:4] <= testData[7:4] + 1 ;
					delayCounter2 <= 0;
					end
				end
				
				if(prevEncoderPulse3 == 1 && encoderPulse3 == 0)
				begin
					if(delayCounter3 > 31'd1_950_000)
					begin
					testData[3:0] <= testData[3:0] + 1 ;
					delayCounter3 <= 0;
					end
				end
			end
		
end

always @(posedge sys_clk)
begin
	prevEncoderPulse1 <= encoderPulse1;
	prevEncoderPulse2 <= encoderPulse2;
	prevEncoderPulse3 <= encoderPulse3;
end

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
					if( delayCounter == 12_000_000 ) 
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
//// This module is used to ensure each switch is mapped to the correct LED, by re-arranging these numbers, 
//switches can be mapped to their respective LED Outputs
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
/////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

module displayDriver(
input [11:0] charArray,
output reg [23:0] encodedOp );

    function [7:0] encodeHexDigit;
    input [3:0] hex;
    begin
		case (hex)
      4'h0: encodeHexDigit = 8'b00001001;
      4'h1: encodeHexDigit = 8'b10111101;
      4'h2: encodeHexDigit = 8'b00010011;
      4'h3: encodeHexDigit = 8'b10010001;
      4'h4: encodeHexDigit = 8'b10100101;
      4'h5: encodeHexDigit = 8'b11000001;
      4'h6: encodeHexDigit = 8'b01000001;
      4'h7: encodeHexDigit = 8'b10011101;
      4'h8: encodeHexDigit = 8'b00000001;
      4'h9: encodeHexDigit = 8'b10000101;
      4'hA: encodeHexDigit = 8'b01110001;
      4'hB: encodeHexDigit = 8'b01100001;
      4'hC: encodeHexDigit = 8'b01110011;
      4'hD: encodeHexDigit = 8'b00110001;
      4'hE: encodeHexDigit = 8'b01000011;
      4'hF: encodeHexDigit = 8'b01000111;
      default: encodeHexDigit = 8'b11111111;
      endcase
	end
	endfunction


always @(*)
begin
	 encodedOp[23:16] = encodeHexDigit(charArray[11:8]);
	 encodedOp[15:8]  = encodeHexDigit(charArray[7:4]);
	 encodedOp[7:0]   = encodeHexDigit(charArray[3:0]);
end
endmodule


/////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////



module shifter24bit(
	input sys_clk,rst,
	input [23:0] data_input,
	output reg shift_pulse,latch_clk,sdata );

parameter RESET = 2'b00;
parameter IDLE = 2'b01;
parameter SHIFT = 2'b10;
parameter LATCH = 2'b11;

reg [39:0] dataTOshift = (40'd0);
reg [4:0] index = 5'b000000;
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
					if(index == 5'd23)
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



/////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
module EncrypterModule( 
	input [39:0] keyInputs,
	input [3:0] KeyHex1,KeyHex2,KeyHex3,
	output reg [39:0] keyOuputs
	);


	// Inputs
	reg [39:0] INP_Rightward_1In;
	reg [39:0] INP_Rightward_2In;
	reg [39:0] INP_Rightward_3In;
	reg [39:0] INP_Rightward_RefIn;

	reg [39:0] INP_Leftward_1In;
	reg [39:0] INP_Leftward_2In;
	reg [39:0] INP_Leftward_3In;

	
	wire [39:0] Rightward_1_to_2;
	wire [39:0] Rightward_2_to_3;
	wire [39:0] Rightward_3_to_Ref;

	wire [39:0] Leftward_Ref_to_3;
	wire [39:0] Leftward_3_to_2;
	wire [39:0] Leftward_2_to_1;
	wire [39:0] Leftward_1_to_OP;

	integer i,j,k,p,q,r;
	always @(*)
	begin
	
		//// Rotor 1
		
		for(i=0;i<40;i=i+1) begin
		 INP_Rightward_1In[i]  <= ( (i + KeyHex1) > 39 )  ? keyInputs[(i+KeyHex1-40)] : keyInputs[(i+KeyHex1)] ; 
		end
		
		for(j=0;j<40;j=j+1) begin
		 keyOuputs[j]  <= ( (40 - KeyHex1 + j) > 39 )  ? Leftward_1_to_OP[(j-KeyHex1)] : Leftward_1_to_OP[(40 - KeyHex1 + j)] ; 
		end
		
		/// Rotor 2
		
		for(k=0;k<40;k=k+1) begin
		 INP_Rightward_2In[k]  <= ( (k + KeyHex2) > 39 )  ? Rightward_1_to_2[(k+KeyHex2-40)] : Rightward_1_to_2[(k+KeyHex2)] ; 
		end
		
		for(p=0;p<40;p=p+1) begin
		 INP_Leftward_1In[p]  <= ( (40 - KeyHex2 + p) > 39 )  ? Leftward_2_to_1[(p-KeyHex2)] : Leftward_2_to_1[(40 - KeyHex2 + p)] ; 
		end
		
		// Rotor 3
		
		for(q=0;q<40;q=q+1) begin
		 INP_Rightward_3In[q]  <= ( (q + KeyHex3) > 39 )  ? Rightward_2_to_3[(q+KeyHex3-40)] : Rightward_2_to_3[(q+KeyHex3)] ; 
		end
		
		for(r=0;r<40;r=r+1) begin
		 INP_Leftward_2In[r]  <= ( (40 - KeyHex3 + r) > 39 )  ? Leftward_3_to_2[(r-KeyHex3)] : Leftward_3_to_2[(40 - KeyHex3 + r)] ; 
		end
		
		INP_Rightward_RefIn <= Rightward_3_to_Ref;
		INP_Leftward_3In <= Leftward_Ref_to_3;

	end


	// Instantiate the ROTORS AS (UUT)
	Rotor1 uut1 (
		.inputLeft(INP_Rightward_1In), 
		.inputRight(INP_Leftward_1In), 
		.outputLeft(Leftward_1_to_OP), 
		.outputRight(Rightward_1_to_2)
	);
	
	Rotor2 uut2 (
		.inputLeft(INP_Rightward_2In), 
		.inputRight(INP_Leftward_2In), 
		.outputLeft(Leftward_2_to_1), 
		.outputRight(Rightward_2_to_3)
	);
	
	Rotor3 uut3 (
		.inputLeft(INP_Rightward_3In), 
		.inputRight(INP_Leftward_3In), 
		.outputLeft(Leftward_3_to_2), 
		.outputRight(Rightward_3_to_Ref)
	);
	
	
	Reflector uut (
		.inputLeft(INP_Rightward_RefIn), 
		.outputLeft(Leftward_Ref_to_3)
	);

endmodule
///////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
module Rotor1(
	input [39:0] inputLeft,
	input [39:0] inputRight,
	output [39:0] outputLeft,
	output [39:0] outputRight
	 );

assign outputRight[0] = inputLeft[22];		assign outputLeft[22] = inputRight[0];
assign outputRight[1] = inputLeft[16];		assign outputLeft[16] = inputRight[1];
assign outputRight[2] = inputLeft[34];		assign outputLeft[34] = inputRight[2];
assign outputRight[3] = inputLeft[24];		assign outputLeft[24] = inputRight[3];
assign outputRight[4] = inputLeft[8];		assign outputLeft[8] = inputRight[4];
assign outputRight[5] = inputLeft[12];		assign outputLeft[12] = inputRight[5];
assign outputRight[6] = inputLeft[5];		assign outputLeft[5] = inputRight[6];
assign outputRight[7] = inputLeft[10];		assign outputLeft[10] = inputRight[7];
assign outputRight[8] = inputLeft[4];		assign outputLeft[4] = inputRight[8];
assign outputRight[9] = inputLeft[33];		assign outputLeft[33] = inputRight[9];
assign outputRight[10] = inputLeft[28];	assign outputLeft[28] = inputRight[10];
assign outputRight[11] = inputLeft[38];	assign outputLeft[38] = inputRight[11];
assign outputRight[12] = inputLeft[19];	assign outputLeft[19] = inputRight[12];
assign outputRight[13] = inputLeft[14];	assign outputLeft[14] = inputRight[13];
assign outputRight[14] = inputLeft[32];	assign outputLeft[32] = inputRight[14];
assign outputRight[15] = inputLeft[30];	assign outputLeft[30] = inputRight[15];
assign outputRight[16] = inputLeft[27];	assign outputLeft[27] = inputRight[16];
assign outputRight[17] = inputLeft[11];	assign outputLeft[11] = inputRight[17];
assign outputRight[18] = inputLeft[37];	assign outputLeft[37] = inputRight[18];
assign outputRight[19] = inputLeft[2];		assign outputLeft[2] = inputRight[19];
assign outputRight[20] = inputLeft[17];	assign outputLeft[17] = inputRight[20];
assign outputRight[21] = inputLeft[39];	assign outputLeft[39] = inputRight[21];
assign outputRight[22] = inputLeft[29];	assign outputLeft[29] = inputRight[22];
assign outputRight[23] = inputLeft[36];	assign outputLeft[36] = inputRight[23];
assign outputRight[24] = inputLeft[23];	assign outputLeft[23] = inputRight[24];
assign outputRight[25] = inputLeft[9];		assign outputLeft[9] = inputRight[25];
assign outputRight[26] = inputLeft[0];		assign outputLeft[0] = inputRight[26];
assign outputRight[27] = inputLeft[3];		assign outputLeft[3] = inputRight[27];
assign outputRight[28] = inputLeft[20];	assign outputLeft[20] = inputRight[28];
assign outputRight[29] = inputLeft[31];	assign outputLeft[31] = inputRight[29];
assign outputRight[30] = inputLeft[15];	assign outputLeft[15] = inputRight[30];
assign outputRight[31] = inputLeft[25];	assign outputLeft[25] = inputRight[31];
assign outputRight[32] = inputLeft[13];	assign outputLeft[13] = inputRight[32];
assign outputRight[33] = inputLeft[21];	assign outputLeft[21] = inputRight[33];
assign outputRight[34] = inputLeft[18];	assign outputLeft[18] = inputRight[34];
assign outputRight[35] = inputLeft[1];		assign outputLeft[1] = inputRight[35];
assign outputRight[36] = inputLeft[26];	assign outputLeft[26] = inputRight[36];
assign outputRight[37] = inputLeft[35];	assign outputLeft[35] = inputRight[37];
assign outputRight[38] = inputLeft[6];		assign outputLeft[6] = inputRight[38];
assign outputRight[39] = inputLeft[7];		assign outputLeft[7] = inputRight[39];




endmodule
//////////////////////////

module Rotor2(
	input [39:0] inputLeft,
	input [39:0] inputRight,
	output [39:0] outputLeft,
	output [39:0] outputRight
    );

assign outputRight[0] = inputLeft[26];		assign outputLeft[26] = inputRight[0];
assign outputRight[1] = inputLeft[7];		assign outputLeft[7] = inputRight[1];
assign outputRight[2] = inputLeft[32];		assign outputLeft[32] = inputRight[2];
assign outputRight[3] = inputLeft[9];		assign outputLeft[9] = inputRight[3];
assign outputRight[4] = inputLeft[3];		assign outputLeft[3] = inputRight[4];
assign outputRight[5] = inputLeft[21];		assign outputLeft[21] = inputRight[5];
assign outputRight[6] = inputLeft[28];		assign outputLeft[28] = inputRight[6];
assign outputRight[7] = inputLeft[25];		assign outputLeft[25] = inputRight[7];
assign outputRight[8] = inputLeft[14];		assign outputLeft[14] = inputRight[8];
assign outputRight[9] = inputLeft[1];		assign outputLeft[1] = inputRight[9];
assign outputRight[10] = inputLeft[8];		assign outputLeft[8] = inputRight[10];
assign outputRight[11] = inputLeft[0];		assign outputLeft[0] = inputRight[11];
assign outputRight[12] = inputLeft[23];	assign outputLeft[23] = inputRight[12];
assign outputRight[13] = inputLeft[19];	assign outputLeft[19] = inputRight[13];
assign outputRight[14] = inputLeft[10];	assign outputLeft[10] = inputRight[14];
assign outputRight[15] = inputLeft[20];	assign outputLeft[20] = inputRight[15];
assign outputRight[16] = inputLeft[13];	assign outputLeft[13] = inputRight[16];
assign outputRight[17] = inputLeft[35];	assign outputLeft[35] = inputRight[17];
assign outputRight[18] = inputLeft[36];	assign outputLeft[36] = inputRight[18];
assign outputRight[19] = inputLeft[31];	assign outputLeft[31] = inputRight[19];
assign outputRight[20] = inputLeft[4];		assign outputLeft[4] = inputRight[20];
assign outputRight[21] = inputLeft[37];	assign outputLeft[37] = inputRight[21];
assign outputRight[22] = inputLeft[11];	assign outputLeft[11] = inputRight[22];
assign outputRight[23] = inputLeft[18];	assign outputLeft[18] = inputRight[23];
assign outputRight[24] = inputLeft[6];		assign outputLeft[6] = inputRight[24];
assign outputRight[25] = inputLeft[39];	assign outputLeft[39] = inputRight[25];
assign outputRight[26] = inputLeft[16];	assign outputLeft[16] = inputRight[26];
assign outputRight[27] = inputLeft[33];	assign outputLeft[33] = inputRight[27];
assign outputRight[28] = inputLeft[22];	assign outputLeft[22] = inputRight[28];
assign outputRight[29] = inputLeft[5];		assign outputLeft[5] = inputRight[29];
assign outputRight[30] = inputLeft[24];	assign outputLeft[24] = inputRight[30];
assign outputRight[31] = inputLeft[38];	assign outputLeft[38] = inputRight[31];
assign outputRight[32] = inputLeft[29];	assign outputLeft[29] = inputRight[32];
assign outputRight[33] = inputLeft[2];		assign outputLeft[2] = inputRight[33];
assign outputRight[34] = inputLeft[34];	assign outputLeft[34] = inputRight[34];
assign outputRight[35] = inputLeft[12];	assign outputLeft[12] = inputRight[35];
assign outputRight[36] = inputLeft[17];	assign outputLeft[17] = inputRight[36];
assign outputRight[37] = inputLeft[15];	assign outputLeft[15] = inputRight[37];
assign outputRight[38] = inputLeft[27];	assign outputLeft[27] = inputRight[38];
assign outputRight[39] = inputLeft[30];	assign outputLeft[30] = inputRight[39];


endmodule
////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
module Rotor3(
	input [39:0] inputLeft,
	input [39:0] inputRight,
	output [39:0] outputLeft,
	output [39:0] outputRight
    );

assign outputRight[0] = inputLeft[9];		assign outputLeft[9] = inputRight[0];
assign outputRight[1] = inputLeft[39];		assign outputLeft[39] = inputRight[1];
assign outputRight[2] = inputLeft[13];		assign outputLeft[13] = inputRight[2];
assign outputRight[3] = inputLeft[14];		assign outputLeft[14] = inputRight[3];
assign outputRight[4] = inputLeft[26];		assign outputLeft[26] = inputRight[4];
assign outputRight[5] = inputLeft[21];		assign outputLeft[21] = inputRight[5];
assign outputRight[6] = inputLeft[6];		assign outputLeft[6] = inputRight[6];
assign outputRight[7] = inputLeft[34];		assign outputLeft[34] = inputRight[7];
assign outputRight[8] = inputLeft[38];		assign outputLeft[38] = inputRight[8];
assign outputRight[9] = inputLeft[37];		assign outputLeft[37] = inputRight[9];
assign outputRight[10] = inputLeft[18];	assign outputLeft[18] = inputRight[10];
assign outputRight[11] = inputLeft[28];	assign outputLeft[28] = inputRight[11];
assign outputRight[12] = inputLeft[11];	assign outputLeft[11] = inputRight[12];
assign outputRight[13] = inputLeft[5];		assign outputLeft[5] = inputRight[13];
assign outputRight[14] = inputLeft[17];	assign outputLeft[17] = inputRight[14];
assign outputRight[15] = inputLeft[16];	assign outputLeft[16] = inputRight[15];
assign outputRight[16] = inputLeft[1];		assign outputLeft[1] = inputRight[16];
assign outputRight[17] = inputLeft[36];	assign outputLeft[36] = inputRight[17];
assign outputRight[18] = inputLeft[20];	assign outputLeft[20] = inputRight[18];
assign outputRight[19] = inputLeft[2];		assign outputLeft[2] = inputRight[19];
assign outputRight[20] = inputLeft[10];	assign outputLeft[10] = inputRight[20];
assign outputRight[21] = inputLeft[23];	assign outputLeft[23] = inputRight[21];
assign outputRight[22] = inputLeft[7];		assign outputLeft[7] = inputRight[22];
assign outputRight[23] = inputLeft[8];		assign outputLeft[8] = inputRight[23];
assign outputRight[24] = inputLeft[25];	assign outputLeft[25] = inputRight[24];
assign outputRight[25] = inputLeft[35];	assign outputLeft[35] = inputRight[25];
assign outputRight[26] = inputLeft[31];	assign outputLeft[31] = inputRight[26];
assign outputRight[27] = inputLeft[0];		assign outputLeft[0] = inputRight[27];
assign outputRight[28] = inputLeft[4];		assign outputLeft[4] = inputRight[28];
assign outputRight[29] = inputLeft[24];	assign outputLeft[24] = inputRight[29];
assign outputRight[30] = inputLeft[12];	assign outputLeft[12] = inputRight[30];
assign outputRight[31] = inputLeft[27];	assign outputLeft[27] = inputRight[31];
assign outputRight[32] = inputLeft[29];	assign outputLeft[29] = inputRight[32];
assign outputRight[33] = inputLeft[30];	assign outputLeft[30] = inputRight[33];
assign outputRight[34] = inputLeft[22];	assign outputLeft[22] = inputRight[34];
assign outputRight[35] = inputLeft[33];	assign outputLeft[33] = inputRight[35];
assign outputRight[36] = inputLeft[19];	assign outputLeft[19] = inputRight[36];
assign outputRight[37] = inputLeft[3];		assign outputLeft[3] = inputRight[37];
assign outputRight[38] = inputLeft[32];	assign outputLeft[32] = inputRight[38];
assign outputRight[39] = inputLeft[15];	assign outputLeft[15] = inputRight[39];


endmodule
///////////////////////////////////////////////////////////////////////////


module Reflector(
	input [39:0] inputLeft,
	output [39:0] outputLeft
       );

assign outputLeft[0] = inputLeft[21];
assign outputLeft[1] = inputLeft[39];
assign outputLeft[2] = inputLeft[29];
assign outputLeft[3] = inputLeft[17];
assign outputLeft[4] = inputLeft[31];
assign outputLeft[5] = inputLeft[23];
assign outputLeft[6] = inputLeft[19];
assign outputLeft[7] = inputLeft[24];
assign outputLeft[8] = inputLeft[18];
assign outputLeft[9] = inputLeft[33];
assign outputLeft[10] = inputLeft[36];
assign outputLeft[11] = inputLeft[37];
assign outputLeft[12] = inputLeft[32];
assign outputLeft[13] = inputLeft[30];
assign outputLeft[14] = inputLeft[27];
assign outputLeft[15] = inputLeft[35];
assign outputLeft[16] = inputLeft[20];
assign outputLeft[17] = inputLeft[3];
assign outputLeft[18] = inputLeft[8];
assign outputLeft[19] = inputLeft[6];
assign outputLeft[20] = inputLeft[16];
assign outputLeft[21] = inputLeft[0];
assign outputLeft[22] = inputLeft[28];
assign outputLeft[23] = inputLeft[5];
assign outputLeft[24] = inputLeft[7];
assign outputLeft[25] = inputLeft[26];
assign outputLeft[26] = inputLeft[25];
assign outputLeft[27] = inputLeft[14];
assign outputLeft[28] = inputLeft[22];
assign outputLeft[29] = inputLeft[2];
assign outputLeft[30] = inputLeft[13];
assign outputLeft[31] = inputLeft[4];
assign outputLeft[32] = inputLeft[12];
assign outputLeft[33] = inputLeft[9];
assign outputLeft[34] = inputLeft[38];
assign outputLeft[35] = inputLeft[15];
assign outputLeft[36] = inputLeft[10];
assign outputLeft[37] = inputLeft[11];
assign outputLeft[38] = inputLeft[34];
assign outputLeft[39] = inputLeft[1];


endmodule


////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////
