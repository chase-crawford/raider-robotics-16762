package org.firstinspires.ftc.teamcode.Year24_25.ActiveUse;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="PIDF Calibration")

public class PIDF_Calibration extends OpMode{

	private enum DriveStyle{
		FORWARD,
		STRAFE,
		TURN;
	}

	private DriveStyle driveDir = DriveStyle.FORWARD;

	private DcMotorEx motor0, motor1, motor2, motor3;

	private DcMotorEx viper0, viper1;

	private Servo sElbow0, sClaw1;
	private boolean clawClosed, elbowDown;

	private DcMotorEx[] motors = new DcMotorEx[4];

	private boolean prevTargetIncrease, prevTargetDecrease, prevPowerToggle;
	private boolean prevPIDFLeft, prevPIDFRight, prevPIDFIncrease, prevPIDFDecrease;
	private boolean prevDiffDecrease, prevDiffIncrease;
	private boolean isDone;
	private boolean prevForward, prevStrafe, prevTurn;
	private boolean prevElbowToggle, prevClawToggle;
	private boolean prevLiftHome, prevLiftHang;

	private double[] pidfs = {InitVars.p, InitVars.i, InitVars.d, InitVars.f};
	private int i;

	private int forwardPos;
	private int strafePos;
	private int turnPos;

	private double power;
	private int diff = 100;

	private InitVars iv;

	@Override
	public void init(){
		initDrivingMotors();
		initLift();
		initElbowAndClaw();

		motors[0] = motor0;
		motors[1] = motor1;
		motors[2] = motor2;
		motors[3] = motor3;

		updatePIDFs();

		iv = new InitVars();

		telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
	}

	@Override
	public void loop(){
		// Change driving direction on input
			if (gamepad1.dpad_up && !prevForward)
				driveDir = DriveStyle.FORWARD;
			if (gamepad1.dpad_right && !prevStrafe)
				driveDir = DriveStyle.STRAFE;
			if (gamepad1.dpad_down && !prevTurn)
				driveDir = DriveStyle.TURN;

		// change motor driving target position
			if (gamepad1.left_stick_y < -.5 && !prevTargetIncrease)
				updateTargets(diff);
			if (gamepad1.left_stick_y > .5 && !prevTargetDecrease)
				updateTargets(-diff);

		// toggle power to motors on and off
			if (gamepad1.left_stick_button && !prevPowerToggle){
				if (power != 0)
					power = 0;
				else
					power = iv.autoDriveSpeed;

				updatePowers();
			}

		// Change current PIDF coefficient value
			if (gamepad2.dpad_up && !prevPIDFIncrease){
				pidfs[i]+=.25;

				updatePIDFs();
			}
			if (gamepad2.dpad_down && !prevPIDFDecrease){
				pidfs[i]-=.25;

				pidfs[i] = pidfs[i]<0 ? 0 : pidfs[i];

				updatePIDFs();
			}

		// Change current PIDF coefficient index
			if (gamepad2.dpad_left && !prevPIDFLeft){
				i--;
				i = i<0 ? 0 : i;
			}
			if (gamepad2.dpad_right && !prevPIDFRight){
				i++;
				i = i>3 ? 3 : i;
			}

		// Increase difference between each target change
			if (gamepad1.right_stick_y > .5 && !prevDiffDecrease){
				diff -= 50;
			}
			if (gamepad1.right_stick_y < -.5 && !prevDiffIncrease){
				diff += 50;
			}
			
		// handle fine tune viper up and down
			float lift = (gamepad1.right_trigger - gamepad1.left_trigger);
			int newTarget = (int)(viper0.getTargetPosition() + lift*25);
			
			if (newTarget < iv.MAX_HEIGHT && newTarget > iv.MIN_HEIGHT){
				 viper0.setTargetPosition(newTarget);
				 viper0.setMode(DcMotor.RunMode.RUN_TO_POSITION);

				 viper1.setTargetPosition(newTarget);
				 viper1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			}
			
		// handle preset positions for lift
			if(gamepad1.a && !prevLiftHome){		
				// toggle between home position and slightly above submersible lip -CC
				if (viper0.getTargetPosition() == iv.BOTTOM_PRESET)
					sendLiftTo(iv.VIPER_HOME, iv.altLiftSpeed);
				
				else
					sendLiftTo(iv.BOTTOM_PRESET, iv.altLiftSpeed);
			}
			
			
			if(gamepad1.b && !prevLiftHang){								//used to be VIPER_HOME
				if (viper0.getTargetPosition() == iv.HANG_PRESET)
					sendLiftTo(iv.VIPER_HOME, iv.altLiftSpeed);
				
				else
					sendLiftTo(iv.HANG_PRESET, iv.altLiftSpeed);
			}
			
			if(gamepad1.y)
				sendLiftTo(iv.TOP_PRESET);
			
			if(gamepad1.x)
				sendLiftTo(iv.MID_PRESET);
				
			if (gamepad1.left_bumper && !prevClawToggle)
				toggleClaw();
			
			if (gamepad1.right_bumper && !prevElbowToggle)
				toggleElbow();

		// Check if any motors are still moving to target position
			isDone = motor0.getCurrentPosition() == motor0.getTargetPosition()
					&& motor1.getCurrentPosition() == motor1.getTargetPosition()
					&& motor3.getCurrentPosition() == motor3.getTargetPosition()
					&& motor2.getCurrentPosition() == motor2.getTargetPosition();

		// Telemetry output for info
			telemetry.addData("Current Power", power);
			telemetry.addData("Is Done Moving", isDone);
			telemetry.addData("Current Position", String.format("[%,d, %,d, %,d, %,d]",
																motor3.getCurrentPosition(), motor0.getCurrentPosition(),
																motor1.getCurrentPosition(), motor2.getCurrentPosition()));
			telemetry.addLine();
			telemetry.addData("Current Drive Direction", driveDir);
			telemetry.addData("Diff amount", diff);
			telemetry.addData("Forward Target", forwardPos);
			telemetry.addData("Strafe Target", strafePos);
			telemetry.addData("Turn Target", turnPos);
			telemetry.addLine();
			telemetry.addData("Current PIDF Selected [0,3]", i);
			telemetry.addData("PIDFS", String.format("[%.2f, %.2f, %.2f, %.2f]",
											pidfs[0], pidfs[1], pidfs[2], pidfs[3]));


		updateButtons();
	}

	public void initDrivingMotors(){
		motor0 = hardwareMap.get(DcMotorEx.class, "motor0");
		motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
		motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
		motor3 = hardwareMap.get(DcMotorEx.class, "motor3");

		motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		motor0.setTargetPosition(0);
		motor1.setTargetPosition(0);
		motor2.setTargetPosition(0);
		motor3.setTargetPosition(0);

		motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		motor0.setDirection(DcMotorSimple.Direction.REVERSE);
		motor3.setDirection(DcMotorSimple.Direction.REVERSE);
	}

	public void updatePIDFs(){
		for (DcMotorEx motor : motors){
			motor.setVelocityPIDFCoefficients(pidfs[0], pidfs[1], pidfs[2], pidfs[3]);
		}
	}

	public void updateTargets(int diff){
		resetEncoders();

		switch (driveDir){
			case FORWARD:
				drive(diff);
				forwardPos += diff;
			break;

			case STRAFE:
				strafe(diff);
				strafePos += diff;
			break;

			case TURN:
				turn(diff);
				turnPos += diff;
			break;

			default:
		}
	}

	public void resetEncoders(){
		for (DcMotorEx motor : motors){
			motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

			motor.setTargetPosition(0);

			motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		}
	}

	public void drive (int diff){
		for (DcMotorEx motor : motors){
			motor.setTargetPosition(diff);
		}
	}

	public void strafe (int diff){
		for (int i=0; i<motors.length; i++){
			if (i % 2 == 0)
				motors[i].setTargetPosition(diff);
			else
				motors[i].setTargetPosition(-diff);
		}
	}

	public void turn (int diff){
		for (int i=0; i<motors.length; i++){
			if (i%3 == 0)
				motors[i].setTargetPosition(diff);
			else
				motors[i].setTargetPosition(-diff);
		}
	}

	public void updatePowers(){
		for (DcMotorEx motor : motors){
			motor.setPower(power);
		}
	}

	public void updateButtons(){
		prevTargetIncrease = gamepad1.left_stick_y < -.5;
		prevTargetDecrease = gamepad1.left_stick_y > .5;
		prevPowerToggle = gamepad1.left_stick_button;

		prevPIDFLeft = gamepad2.dpad_left;
		prevPIDFRight = gamepad2.dpad_right;
		prevPIDFIncrease = gamepad2.dpad_up;
		prevPIDFDecrease = gamepad2.dpad_down;

		prevDiffDecrease = gamepad1.right_stick_y > .5;
		prevDiffIncrease = gamepad1.right_stick_y < -.5;

		prevForward = gamepad1.dpad_up;
		prevStrafe = gamepad1.dpad_right;
		prevTurn = gamepad1.dpad_down;

		prevLiftHome = gamepad1.a;
		prevLiftHang = gamepad1.b;
		
		prevClawToggle = gamepad1.left_bumper;
		prevElbowToggle = gamepad1.right_bumper;
	}

	private void initLift(){
		//Get motor information -CS
			viper0 = hardwareMap.get(DcMotorEx.class, "viper0");
			viper1 = hardwareMap.get(DcMotorEx.class, "viper1");
			
			viper0.setDirection(DcMotorSimple.Direction.FORWARD);
			viper1.setDirection(DcMotorSimple.Direction.REVERSE);
			
			viper0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			viper1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			
			viper0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			viper1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			
			viper0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			viper1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			
		//Send motors to home position and turn them off -CS
			viper0.setTargetPosition(iv.VIPER_HOME);
			viper1.setTargetPosition(iv.VIPER_HOME);
			
			viper0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			viper1.setMode(DcMotor.RunMode.RUN_TO_POSITION);   
			
			viper0.setPower(0);
			viper1.setPower(0);
	}

	private void initElbowAndClaw(){
		sElbow0 = hardwareMap.get(Servo.class, "sElbow0");
		sElbow0.setDirection(Servo.Direction.REVERSE);
		sElbow0.setPosition(iv.ELBOW_UP);
			
		sClaw1 = hardwareMap.get(Servo.class, "sClaw1");
		sClaw1.setPosition(iv.CLAW_OPEN);
	}
	
	/*Allows you to toggle elbow button in stead of holding it down -CS
		@return void
	*/
		public void toggleElbow(){
			//sElbow0.setPosition(sElbow0.getPosition() + 0.02);		//Testing elbow pos by incrementing it +.02 w/ each input -CS
			if (elbowDown){
				sElbow0.setPosition(iv.ELBOW_UP);
				elbowDown = false;
			} 
			
			else{
				sElbow0.setPosition(iv.ELBOW_GRAB);
				elbowDown = true;
			}
		}
	
	
	/*Allows you to toggle claw button in stead of holding it down -CS
		@return void
	*/
		public void toggleClaw(){
			//sClaw1.setPosition(sClaw1.getPosition()+.02);		//Testing claw pos by incrementing it +.02 w/ each input -CS
			if (clawClosed) {
				sClaw1.setPosition(iv.CLAW_OPEN);
				clawClosed = false;
			} 
			
			else{
				sClaw1.setPosition(iv.CLAW_CLOSE);
				clawClosed = true;
			}
		}
	
	/*Set the position of both lift motors, set their speed, and send them 
	  to the position. Automatically sets the motors to iv.liftSpeed -CS
			@return void
	*/
		public void sendLiftTo(int pos){
			viper0.setPower(iv.liftSpeed);
			viper1.setPower(iv.liftSpeed);
			
			viper0.setTargetPosition(pos);
			viper1.setTargetPosition(pos);
		}
	
	
	/*Set the position of both lift motors, set their speed, and send them 
	  to the position. Allows you to set the motors to a specific speed -CS
		@return void
	*/
		public void sendLiftTo(int pos, double speed){
			viper0.setPower(speed);
			viper1.setPower(speed);
			
			viper0.setTargetPosition(pos);
			viper1.setTargetPosition(pos);
		}
}
