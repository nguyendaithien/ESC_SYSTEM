
module ESC( 
intput clk,
 intput reset,
intput wire [3:0] a_x,
 input wire [3:0] a_y,
 input wire [3:0] a_z,
 intput wire [3:0] b_x,
 intput wire [3:0] b_y,
 input wire [3:0] b_z,
 intput wire [3:0] _distance,
 outtput wire [3:0] distance,
 output reg [3:0] accelerometerX,
 output reg [3:0] accelerometerX,
 output reg [3:0] accelerometerX,
 output reg [3:0] gyroscopeX,
 output reg [3:0] gyroscopeY,
 output reg [3:0] gyroscopeZ);

always @( posedge clk or negedge reset) 
	begin 
		if(reset) begin
			accelerometerX = 4'b0000;
			accelerometerY = 4'b0000;
			accelerometerZ = 4'b0000;
			gyroscopeX  = 4'b0000;
			gyroscopeX = 4'b0000;
			gyroscopeX = 4'b0000;
		end
		else begin
			accelerometerX = a_x;
			accelerometerY = a_y;
			accelerometerZ = a_z;
			gyroscopeX = b_x;
			gyroscopeY = b_y;
			gyroscopeZ = b_z;
		end
	end
endmodule

module VL53L1( input wire [3:0] _distance, output reg [3:0] distance );
always @( posedge clk or negedge reset) 
	begin 
		if( reset) begin 
			distance = 4'b0000;
		end
		else begin
			distance = _distance;
		end
	end
endmodule

module controler( intput wire clk,
intput wire reset,
 outtput wire [3:0] distance,
 input reg [3:0] accelerometerX,
 input reg [3:0] accelerometerY,
 input reg [3:0] accelerometerZ,
 input reg [3:0] gyroscopeX,
 input reg [3:0] gyroscopeY,
 input reg [3:0] gyroscopeZ,
 output reg [3:0] rollPart,
 output reg [3:0] pitchPart,
 output reg [3:0] yawPart,
  );

  always @(posedge clk or negedge reset )
  begin 
		if( reset ) begin 
			rollPart = 4'b0000;
			pitchPart = 4'b0000;
			yawPart = 4'b0000;
		end 
		else 
		begin 
			rollPart =  accelerometerX & gyroscopeX;
			yawPart = accelerometerY & gyroscopeY;
			pitchPart = accelerometerY & gyroscopeZ;
		end
  end
  endmodule
module PWM( input wire[3:0] rollPart,
input wire[3:0] pitchPart,
input wire[3:0] pitchPart,
output reg[3:0] motorForces_1,
output reg[3:0] motorForces_2,
output reg[3:0] motorForces_3,
output reg[3:0] motorForces_4);





  motorForces[0] = thrustPart - rollPart - pitchPart - yawPart;
  motorForces[1] = thrustPart - rollPart + pitchPart + yawPart;
  motorForces[2] = thrustPart + rollPart + pitchPart - yawPart;
  motorForces[3] = thrustPart + rollPart - pitchPart + yawPart;
    const float arm = 0.707106781f * armLength;
  const float rollPart = 0.25f / arm *  torque->x;
  const float pitchPart = 0.25f / arm *  torque->y;
  const float thrustPart = 0.25f *  thrust; // N (per rotor)
  const float yawPart = 0.25f *  torque->y / thrustToTorque;