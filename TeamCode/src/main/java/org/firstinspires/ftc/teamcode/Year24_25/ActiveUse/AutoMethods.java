package org.firstinspires.ftc.teamcode.Year24_25.ActiveUse;

import com.acmerobotics.roadrunner.InstantFunction;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class AutoMethods extends OpMethods{

	/*
    These next few InstantFunction classes are used as a way for us to
    pass methods from AutoMethods into the Trajectory/RR Path.
    For now, they are mainly used to move the lift and servos around mid path.
     */

	public class OpenClaw implements InstantFunction {
		@Override
		public void run(){
			openClaw();
		}
	}

	public class CloseClaw implements InstantFunction{
		@Override
		public void run(){
			closeClaw();
		}
	}

	public class RaiseElbow implements InstantFunction{
		@Override
		public void run(){
			raiseElbow();
		}
	}

	public class RaiseElbowToWall implements InstantFunction{
		@Override
		public void run(){raiseElbowToWall();}
	}

	public class LowerElbow implements InstantFunction{
		@Override
		public void run(){
			lowerElbow();
		}
	}

	public class SendLiftTo implements InstantFunction{
		int targetPos;

		public SendLiftTo(int targetPos){
			this.targetPos = targetPos;
		}

		@Override
		public void run(){
			sendLiftTo(targetPos, InitVars.altLiftSpeed);
		}
	}

	public class WaitForLift implements InstantFunction{
		@Override
		public void run(){
			waitForLift();
		}
	}
	
	// var to check if opMode is active
		private LinearOpMode op;
		
	//Create object to access InitVars class -CS
		private InitVars IV = new InitVars();
		
	// Create Telemetry output items
		private Telemetry.Item posTelem;
		private Telemetry.Item targetTelem;
		private Telemetry.Item liftPosTelem;
		private Telemetry.Item liftTargetTelem;
	
	// var for storing Ticks -> CM conversion
		static final double	TICKS_PER_CM = 1.17742528;
		static final int driveErrorMargin = 3;
		static final int liftErrorMargin = 30;
	
	public AutoMethods(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode op){
		super(hardwareMap, telemetry);
		
		this.op = op;
	}

	
	/*Gets the motors from the control hub, sets their direction and their brake behavior.
	  Directions are set so that when all power values are positive, all motors move the
	  robot forwards. -CS
	  Also sets the motors to use encoder values to movep - CC
			@return void
	*/
	@Override
	public void initDriveMotors(){
		
		// call initDriveMotors from OpMethods parent class
			super.initDriveMotors();
		
		// reset encoders and set mode to use encoders
		// also set up PIDF coefficients to make it faster/pinpoint
			for (DcMotorEx motor : motors){
				motor.setTargetPosition(0);
				motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
				motor.setVelocityPIDFCoefficients(InitVars.p, InitVars.i, InitVars.d, InitVars.f);
			}

		posTelem = telemetry.addData("Current Position", getCurrentPositionTelem());
		targetTelem = telemetry.addData("Target Position", getTargetPositionTelem());
	}
	
	@Override
	public void initLiftMotors(){
		super.initLiftMotors();
		
		for (DcMotor motor : lift){
			motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		}

		liftPosTelem = telemetry.addData("Lift Current Position", getLiftCurrentPosition());
		liftTargetTelem = telemetry.addData("Lift TargetPosition", getLiftTargetPosition());
	}

	/* this method will act as the main wiring behind all 
	driveTicks and driveCM methods. This will change the motors by a
	specified delta position and run them to these positions. */
		private void drive(int delta0, int delta1, int delta2, int delta3){
			
			int[] deltas = {delta0, delta1, delta2, delta3};
			
			// update mode and power
				for (int i=0; i<motors.length; i++){
					//motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
					motors[i].setTargetPosition(motors[i].getCurrentPosition()+deltas[i]);
					motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
					motors[i].setPower((float)InitVars.autoDriveSpeed);
				}
				
			// show telemetry and wait till it ends
				while(isDriving() && op.opModeIsActive())
				{
					// telemetry to tell target position and current position
						/*String currPos = String.format("[%,d, %,d, %,d %,d]",
							motors[0].getCurrentPosition(),
							motors[1].getCurrentPosition(),
							motors[2].getCurrentPosition(),
							motors[3].getCurrentPosition());
						String targetPos = String.format("[%,d, %,d, %,d %,d]",
							motors[0].getTargetPosition(),
							motors[1].getTargetPosition(),
							motors[2].getTargetPosition(),
							motors[3].getTargetPosition());
							
						telemetry.addData("Current Position (Ticks)", currPos);
						telemetry.addData("Target Position (Ticks)", targetPos);
						telemetry.update();*/
						
						posTelem.setValue(getCurrentPositionTelem());
						targetTelem.setValue(getTargetPositionTelem());
						telemetry.update();
						
						op.idle();
				}
		}
		
	/* this method will act as the main wiring behind all 
	driveTicks and driveCM methods. This will change all the motors by a
	specified delta position and run them to these new positions, the delta
	all being the same for the motors. */
		public void drive (int delta){
			// update mode and power AND position
				for (DcMotorEx motor : motors){
					//motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
					motor.setTargetPosition(motor.getCurrentPosition()+delta);
					motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
					motor.setPower((float)InitVars.autoDriveSpeed);
				}
				
			// show telemetry and wait till it ends
				while(isDriving() && op.opModeIsActive())
				{
					// telemetry to tell target position and current position
						/*String currPos = String.format("[%,d, %,d, %,d %,d",
							motors[0].getCurrentPosition(),
							motors[1].getCurrentPosition(),
							motors[2].getCurrentPosition(),
							motors[3].getCurrentPosition());
						String targetPos = String.format("[%,d, %,d, %,d %,d",
							motors[0].getTargetPosition(),
							motors[1].getTargetPosition(),
							motors[2].getTargetPosition(),
							motors[3].getTargetPosition());
							
						telemetry.addData("Current Position (Ticks)", currPos);
						telemetry.addData("Target Position (Ticks)", targetPos);
						telemetry.update();*/
						
						posTelem.setValue(getCurrentPositionTelem());
						targetTelem.setValue(getTargetPositionTelem());
						telemetry.update();
						
						op.idle();
				}
		}
		
	/* set the PIDF coefficients to make these motors smoother and more direct
	I dont feel like explaining PIDF it just makes it slow down when it reaches
	and get exactly on the target pos and also makes it fast - CC */
		public void setPIDFs(){
			for (DcMotorEx motor : motors){
				motor.setVelocityPIDFCoefficients(InitVars.p, InitVars.i, InitVars.d, InitVars.f);
			}
		}
		
	// wait for lift to get to its target position
		public void waitForLift(){
			int target = getLiftTargetPosition();
			int current = getLiftCurrentPosition();

			// wait until current position = target position
			while (Math.abs(getLiftTargetPosition() - getLiftCurrentPosition()) > liftErrorMargin){
				// telemetry for target and current position
				/*telemetry.addData("Moving Lift to", target);
				telemetry.addData("Current Position", lift[0].getCurrentPosition());
				telemetry.update();*/

			liftPosTelem.setValue(current);
			liftTargetTelem.setValue(target);
			telemetry.update();

			target = getLiftTargetPosition();
			current = getLiftCurrentPosition();
				
			op.idle();
			}
		}

		public boolean motorsBusy(){
			for (DcMotor motor : motors)
				if (motor.isBusy())
					return true;

			return false;
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

	//
		public void raiseElbowToWall(){sElbow0.setPosition(0.3);}
		
	// raises elbow to 90 degree rotation (arm faces upwards)
		public void lowerElbow(){
			sElbow0.setPosition(InitVars.ELBOW_GRAB);
		}
		
		
	// comment :
		public void startWithSpecimen(){
			sElbow0.setPosition(IV.ELBOW_AUTO_START);
			sClaw1.setPosition(IV.CLAW_CLOSE);
		}

	
	/* The next few methods are all driveCM methods.
	These will function by moving the drive wheels by
	a specified amount through the parameters.
	The parameter units are in CM, per Wilson's estimations - CC */
		public void driveCM(double y, boolean useFineSpeed){
			// Convert
				int d = (int)(y*TICKS_PER_CM);
				
			//Change speed
				if(useFineSpeed){
					for(int i=0; i<motors.length; i++)
						motors[i].setPower(IV.fineAutoDriveSpeed);
				}
			
			drive(d);
		}

		public void driveCM(double x, double y){
			// Convert
				int d0 = (int)((x+y) * TICKS_PER_CM);
				int d1 = (int)((-x+y) * TICKS_PER_CM);
				int d2 = (int)((x+y) * TICKS_PER_CM);
				int d3 = (int)((-x+y) * TICKS_PER_CM);
				
			drive(d0, d1, d2, d3);
		}
		
		public void driveCM(double x, double y, double r){
			// Convert
				int d0 = (int)((x+y+r) * TICKS_PER_CM);
				int d1 = (int)((-x+y-r) * TICKS_PER_CM);
				int d2 = (int)((x+y-r) * TICKS_PER_CM);
				int d3 = (int)((-x+y+r) * TICKS_PER_CM);
				
			drive(d0, d1, d2, d3);
		}
		
	/* The next few methods are all driveTicks methods.
	These will function by moving the drive wheels by
	a specified amount through the parameters.
	The parameter units are in Ticks - CC */
		public void driveTicks(int y){
			drive(y);
		}
		
		public void driveTicks(int x, int y){
			int d0 = x+y;
			int d1 = -x+y;
			int d2 = x+y;
			int d3 = -x+y;
			
			drive(d0, d1, d2, d3);
		}
		
		public void driveTicks(int x, int y, int r){
			int d0 = x+y+r;
			int d1 = -x+y-r;
			int d2 = x+y-r;
			int d3 = -x+y+r;
			
			drive(d0, d1, d2, d3);
		}
	
	/* This method will check if any drive motors have
	not met their current target position. */
		public boolean isDriving(){
			for (DcMotorEx motor : motors){
						if (Math.abs(motor.getCurrentPosition()-motor.getTargetPosition()) > driveErrorMargin) return true;
					}
					
					return false;
		}
		
	/* This method will make sure the robot is facing the proper
	and intended direction/rotation passed through the parameter */
		public void correctRotation(double startRot){							//auto start pos = 0; facing to obsv. = -90; opposite start pos = 180 or -180; facing bins = 90
			if (!isDriving() && op.opModeIsActive()){
				double rotation = compass.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
				
				if (Math.abs(startRot-rotation) > 1){
					int rotationTicks = (int)((startRot-rotation) * 800 / 90);
					
					driveTicks(0,0, -rotationTicks);
				}
			}
		}
	
	/* This method will make sure the robot is facing the proper
	and intended direction/rotation. This rotation is 0, relative
	to the robots starting rotation. */	
		public void correctRotation(){
			if (!isDriving() && op.opModeIsActive()){
				double rotation = compass.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
				
				if (Math.abs(rotation) > 1){
					int rotationTicks = (int)(rotation * 750 / 90);
					
					driveTicks(0, 0, rotationTicks);
				}
			}
		}
		
	// This method will return a string containing all current
	// motor positions
		public String getCurrentPositionTelem(){
			String pos = "[ ";
			
			for (DcMotorEx motor : motors){
				pos += motor.getCurrentPosition()+", ";
			}
			
			return pos + "]";
		}
		
	// This method will return a string containing all target
	// motor positions
		public String getTargetPositionTelem(){
			String pos = "[ ";
			
			for (DcMotorEx motor : motors){
				pos += motor.getTargetPosition()+", ";
			}
			
			return pos + "]";
		}
}





