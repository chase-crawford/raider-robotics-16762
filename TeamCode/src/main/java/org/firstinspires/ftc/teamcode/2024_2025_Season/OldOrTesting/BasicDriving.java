package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import java.text.DecimalFormat;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//@TeleOp(name="Encodering Stuff - CC")

public class BasicDriving extends OpMode{

	private DcMotorEx m0;
	private DcMotorEx m1;
	private DcMotorEx m2;
	private DcMotorEx m3;
	
	private boolean prevDPadUp;
	private boolean prevDPadDown;
	private boolean prevA;
	private boolean prevB;
	
	private float inputDelta = 10;
	private float rawPos;
	
	static final double		COUNTS_PER_MOTOR_REV	 = 1440 ;	 // eg: TETRIX Motor Encoder
	static final double		DRIVE_GEAR_REDUCTION	 = 1.0 ;		// No External Gearing.
	static final double		WHEEL_DIAMETER_CM		= 104.0 ;		// For figuring circumference
	static final double		COUNTS_PER_CM			= 1.17742528;

	@Override
	public void init(){
		
		m0 = hardwareMap.get(DcMotorEx.class, "motor0");
		m1 = hardwareMap.get(DcMotorEx.class, "motor1");
		m2 = hardwareMap.get(DcMotorEx.class, "motor2");
		m3 = hardwareMap.get(DcMotorEx.class, "motor3");

	//Right side motors forward, Left reverse
	  m0.setDirection(DcMotorSimple.Direction.REVERSE);
	  m1.setDirection(DcMotorSimple.Direction.FORWARD);
	  m2.setDirection(DcMotorSimple.Direction.FORWARD);
	  m3.setDirection(DcMotorSimple.Direction.REVERSE);

	  m0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	  m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	  m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	  m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	  
	  m0.setTargetPosition(0);
	  m1.setTargetPosition(0);
	  m2.setTargetPosition(0);
	  m3.setTargetPosition(0);

	  m0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	  m1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	  m2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	  m3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

	  m0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	  m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	  m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	  m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	  
	  //m0.setVelocityPIDFCoefficients(10, 22, 3, 0);
	  //m1.setVelocityPIDFCoefficients(10, 22, 3, 0);
	  //m2.setVelocityPIDFCoefficients(10, 22, 3, 0);
	  //m3.setVelocityPIDFCoefficients(10, 22, 3, 0);
	}

	@Override
	public void loop(){
		int targetDelta = 0;
		
		// move a distance if input is dpad up/down
		if (gamepad1.dpad_up && !prevDPadUp)
			targetDelta += inputDelta;
		if (gamepad1.dpad_down && !prevDPadDown)
			targetDelta -= inputDelta;
			
		// change movement distance per input
		if (gamepad1.a && !prevA)
			inputDelta += 10;
		if (gamepad1.b && !prevA)
			inputDelta -= 10;
			
		// update button press values
		prevDPadUp = gamepad1.dpad_up;
		prevDPadDown = gamepad1.dpad_down;
		prevA = gamepad1.a;
		prevB = gamepad1.b;
		
		int target0 = m0.getTargetPosition();
		int target1 = m1.getTargetPosition();
		int target2 = m2.getTargetPosition();
		int target3 = m3.getTargetPosition();
		
		m0.setTargetPosition((int)(target0+targetDelta*COUNTS_PER_CM));
		m1.setTargetPosition((int)(target1+targetDelta*COUNTS_PER_CM));
		m2.setTargetPosition((int)(target2+targetDelta*COUNTS_PER_CM));
		m3.setTargetPosition((int)(target3+targetDelta*COUNTS_PER_CM));
		
		rawPos += targetDelta;
		
		if (areMotorsBusy()){
			m0.setPower(.5);
			m1.setPower(.5);
			m2.setPower(.5);
			m3.setPower(.5);
		}
		else{
			m0.setPower(0);
			m1.setPower(0);
			m2.setPower(0);
			m3.setPower(0);
		}
		
		telemetry.addData("Target Position", rawPos);
		telemetry.addData("Input Delta", inputDelta);
	}
		
	public boolean areMotorsBusy(){
		return m0.getCurrentPosition() != m0.getTargetPosition() ||
				m1.getCurrentPosition() != m1.getTargetPosition() ||
				m2.getCurrentPosition() != m2.getTargetPosition() ||
				m3.getCurrentPosition() != m3.getTargetPosition();
	}
		
	public void powerMotors(double speed){
	  m0.setPower(speed);
	  m1.setPower(speed);
	  m2.setPower(speed);
	  m3.setPower(speed);
	}

	public void powerMotors(double zero, double one, double two, double three){
	  m0.setPower(zero);
	  m1.setPower(one);
	  m2.setPower(two);
	  m3.setPower(three);
	}

	public double max(double[] powers){
		double ret = powers[0];

		for (int i=0; i<powers.length; i++){
			if (powers[i] > ret){
				ret = powers[i];
			}
		}

		return ret;
	}
	
	//This submethod will output driving motor values
	public void telemetry(double vy, double vx, double w, double fl, double fr,
						  double rl, double rr)
	{
		DecimalFormat df = new DecimalFormat("#,##0.000");
		
		telemetry.addData("Velocity y", df.format(vy));
		telemetry.addData("Velocity x", df.format(vx));
		telemetry.addData("Rotational velocity", df.format(w));
		telemetry.addData("", "");
		
		telemetry.addData("Forward left", df.format(fl));
		telemetry.addData("Forward right", df.format(fr));
		telemetry.addData("Reverse left", df.format(rl));
		telemetry.addData("Reverese right", df.format(rr));
	}

}
