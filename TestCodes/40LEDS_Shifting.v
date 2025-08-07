module Testing3(
	input sys_clk,rst,
	output  shift_pulse,latch_clk,sdata );

	parameter RESET = 2'b00;
	parameter DATA1 = 2'b01;
	parameter DATA2 = 2'b10;
	parameter DATA3 = 2'b11;
	
	
	reg [39:0] testData;
	reg [1:0] state = RESET;
	reg [25:0] delayCounter = 0;
		
	shifter40bit charDisplayUnit(sys_clk,rst,testData,shift_pulse,latch_clk,sdata);

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
						testData <= 40'b0000_0000_0000_0000_0000_0000_0000_0000_0000_0000;
						state <= DATA1;
						end
					DATA1:
						begin
						testData <= 40'b1111_1111_1111_1111_1111_1111_1111_1111_1111_1111;
						state <= DATA2;
						end
					DATA2:
						begin
						testData <= 40'b0101_0101_0101_0101_0101_0101_0101_0101_0101_0101;
						state <= DATA3;
						end
					DATA3:
						begin
						testData <= 40'b1010_1010_1010_1010_1010_1010_1010_1010_1010_1010;
						state <= RESET;
						end
				endcase
			end
		end
		
end
	
endmodule


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