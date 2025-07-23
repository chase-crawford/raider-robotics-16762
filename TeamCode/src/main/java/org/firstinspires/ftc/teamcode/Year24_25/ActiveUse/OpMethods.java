package org.firstinspires.ftc.teamcode.Year24_25.ActiveUse;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BHI260IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;


public class OpMethods {
	// map to get all hardware into vars
		protected HardwareMap hardwareMap;
		
	// Telemetry var for info output
		protected Telemetry telemetry;
	
	// drive mecanum motors
		protected DcMotorEx[] motors;
	
	// lift motors
		protected DcMotor[] lift;
	
	// claw and elbow servos
		protected Servo sElbow0, sClaw1;
		
	// IMU
		protected BHI260IMU compass;
		
	// Constructor to get hardwareMap + telemetry and initialize all hardware vars
		public OpMethods(HardwareMap hardwareMap, Telemetry telemetry){
			this.hardwareMap = hardwareMap;
			this.telemetry = telemetry;
			
			motors = new DcMotorEx[4];
			lift = new DcMotor[2];
		}
		
	/* This method will initialize all hardware fields
	and ready bot for movement */
		public void init(){
			initDriveMotors();
			initLiftMotors();
			initServos();
			
			// set up IMU rotation access
				compass = hardwareMap.get(BHI260IMU.class, "compass");
				compass.resetYaw();
		}
	
	/*Gets the motors from the control hub, sets their direction and their brake behavior.
	  Directions are set so that when all power values are positive, all motors move the
	  robot forwards. -CS
			@return void
	*/
		public void initDriveMotors(){
			// for loop to set zero power behavior and
			// set all positive powers to the same direction
				for (int i=0; i<motors.length; i++){
					motors[i] = hardwareMap.get(DcMotorEx.class, "motor"+i);
					
					motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
					motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
					
					motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
					
					if (i % 3 == 0)
						motors[i].setDirection(DcMotorSimple.Direction.REVERSE);
				}	
		}

	/*Gets the lift motors from the control hub, resets the encoder values, and 
	  sets the motors' direction, encoder behavior, and brake behavior. 
	  Sends the lifts to their home position and then powers off the lifts. -CS
			@return void
	*/
		public void initLiftMotors(){
				for (int i=0; i<lift.length; i++){
					// get motor from hardwareMap
						lift[i] = hardwareMap.get(DcMotorEx.class, "viper"+i);
					
					// reset encoder position so bottom is 0
						//lift[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
					
					// restart mode to follow encoder
						lift[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
					
					// brake motor on zero power so lift doesnt just fall
						lift[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
					
					// make viper target home/start position
						lift[i].setTargetPosition(InitVars.VIPER_HOME);
					
					// restart mode to move to target position
						lift[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
					
					// start power at 0
						lift[i].setPower(0);
					
					// make both lift motors go the same direction
						if (i == 0)
							lift[i].setDirection(DcMotorSimple.Direction.FORWARD);
						else
							lift[i].setDirection(DcMotorSimple.Direction.REVERSE);
				}
		}
	
	/*Gets the elbow and claw servo from the control hub and sets their directions.
	  Sends both servos to their initialization positions -CS
	  Elbow, then Claw! - CC
			@return void
	*/
		public void initServos(){
			sElbow0 = hardwareMap.get(Servo.class, "sElbow0");
			sElbow0.setDirection(Servo.Direction.REVERSE);
			//sElbow0.setPosition(InitVars.ELBOW_UP);
				
			sClaw1 = hardwareMap.get(Servo.class, "sClaw1");
			//sClaw1.setPosition(InitVars.CLAW_OPEN);
		}
	
	/* returns the average of the lift current positions as an integer */
		public int getLiftCurrentPosition(){
			return (int)((lift[1].getCurrentPosition()+lift[0].getCurrentPosition())/2.0);
		}
		
	/* returns the average of the lift current positions as an integer */
		public int getLiftTargetPosition(){
			return (int)((lift[1].getTargetPosition()+lift[0].getTargetPosition())/2);
		}
		
	/*Set the position of both lift motors, set their speed, and send them 
	  to the position. Allows you to set the motors to a specific speed -CS
		@return void
	*/
		public void sendLiftTo(int pos, double speed){
			lift[0].setPower(speed);
			lift[1].setPower(speed);
			
			lift[0].setTargetPosition(pos);
			lift[1].setTargetPosition(pos);
		}
		
	/*Set the position of both lift motors, set their speed, and send them 
	  to the position. Automatically sets the motors to iv.liftSpeed -CS
			@return void
	*/
		public void sendLiftTo(int pos){
			lift[0].setPower(InitVars.liftSpeed);
			lift[1].setPower(InitVars.liftSpeed);
			
			lift[0].setTargetPosition(pos);
			lift[1].setTargetPosition(pos);
		}
		
	/* Set the power of the lift at will simaultaneously - CC*/
		public void setLiftPower(double power){
			lift[0].setPower(power);
			lift[1].setPower(power);
		}
		
	/*Power all motors with specific speeds -CS
		@param pow - the specific speeds for each motor
		@return void
	*/
		public void setDrivePower(double pow){
		  for (DcMotorEx motor : motors){
		  	motor.setPower(pow);
		  }
		}
		
	/*Power all motors with specific speeds -CS
		@param rl, fl, fr, rr - the specific speeds for each respective motor
		@return void
	*/
		public void setDrivePower(double rl, double fl, double fr, double rr){
		  motors[0].setPower(fl);
		  motors[1].setPower(fr);
		  motors[2].setPower(rr);
		  motors[3].setPower(rl);
		}

	// opens the claw up to iv.CLAW_OPEN position
	public void openClaw(){
		sClaw1.setPosition(InitVars.CLAW_OPEN);
	}

	// closes claw to closeClawPos position
	public void closeClaw(){
		sClaw1.setPosition(InitVars.CLAW_CLOSE);
	}

	// raises elbow to 90 degree rotation (arm faces upwards)
	public void raiseElbow(){
		sElbow0.setPosition(InitVars.ELBOW_UP);
	}

	// Raise elbow to 45 degrees(?) rotation to grab from wall
	public void raiseElbowToWall(){
		sElbow0.setPosition(InitVars.ELBOW_GRAB_WALL);
	}

	// raises elbow to 90 degree rotation (arm faces upwards)
	public void lowerElbow(){
		sElbow0.setPosition(InitVars.ELBOW_GRAB);
	}
}



