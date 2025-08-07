module Testing3(
	input sys_clk,rst,
	output  shift_pulse,latch_clk,sdata ,
	output  shift_pulse2,latch_clk2,sdata2);

	parameter RESET = 4'b0000;
	parameter DATA1 = 4'b0001;
	parameter DATA2 = 4'b0010;
	parameter DATA3 = 4'b0011;
	parameter DATA4 = 4'b0100;
	parameter DATA5 = 4'b0101;
	parameter DATA6 = 4'b0110;
	parameter DATA7 = 4'b0111;
	parameter DATA8 = 4'b1000;
	parameter DATA9 = 4'b1001;
	parameter DATA10 = 4'b1010;
	parameter DATA11 = 4'b1011;
	parameter DATA12 = 4'b1100;
	parameter DATA13 = 4'b1101;
	parameter DATA14 = 4'b1110;
	parameter DATA15 = 4'b1111;
	
	reg [11:0] testData;
	reg [3:0] state = RESET;
	reg [25:0] delayCounter = 0;
	
	wire [23:0] decodedop;
	
	displayDriver diplayencoder(testData,decodedop);
	shifter24bit charDisplayUnit(sys_clk,rst,decodedop,shift_pulse2,latch_clk2,sdata2);

always @(posedge sys_clk) begin
	if(rst)begin
	delayCounter <= 0;
	state <= RESET;
	end
	else
		begin
		delayCounter <= delayCounter + 1;
		if (delayCounter == 26'd50000000)
			begin
				delayCounter <= 26'd0 ;
				case (state)
					RESET: 
						begin
						testData <= 12'h010; // Zero
						state <= DATA1;
						end
					DATA1:
						begin
						testData <= 12'h1D1; // 1
						state <= DATA2;
						end
					DATA2:
						begin
						testData <= 12'h2A2; //2
						state <= DATA3;
						end
					DATA3:
						begin
						testData <= 12'h3A3; //3
						state <= DATA4;
						end
					DATA4:
						begin
						testData <= 12'h444; //4
						state <= DATA5;
						end
					DATA5:
						begin
						testData <= 12'h505; //5
						state <= DATA6;
						end
					DATA6:
						begin
						testData <= 12'h666; //6
						state <= DATA7;
						end
					DATA7:
						begin
						testData <= 12'h7B7;//7
						state <= DATA8;
						end
					DATA8:
						begin
						testData <= 12'h8F8; //8
						state <= DATA9;
						end
					DATA9:
						begin
						testData <= 12'h9E9; //9
						state <= DATA10;
						end
					DATA10:
						begin
						testData <= 12'hAAA; //A
						state <= DATA11;
						end
					DATA11:
						begin
						testData <= 12'hB2B; //B
						state <= DATA12;
						end
					DATA12:
						begin
						testData <= 12'hCDC; //C
						state <= DATA13;
						end
					DATA13:
						begin
						testData <= 12'hD9D; //D
						state <= DATA14;
						end
					DATA14:
						begin
						testData <= 12'hE3E; //E
						state <= DATA15;
						end
					DATA15:
						begin
						testData <= 12'hF6F; //F
						state <= RESET;
						end
				endcase
			end
		end
		
end
	
endmodule

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
