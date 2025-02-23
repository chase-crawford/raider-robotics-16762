package org.firstinspires.ftc.teamcode.Year24_25.ActiveUse;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BHI260IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.text.DecimalFormat;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// manually input, do not delete pls - CC


	/* All driving controls on gamepad1
	   All lift, claw, and elbow controls on gamepad2 -CS	*/
	   
//@TeleOp(name="Main_TeleOp - Two Drivers")

public class Main_TwoDrivers extends OpMode{
	//Create object for iv class
		InitVars iv = new InitVars();

	// create sample detector vars and vision portal
		private ColorBlobLocatorProcessor blue;
		private ColorBlobLocatorProcessor red;
		private ColorBlobLocatorProcessor yellow;
		private VisionPortal portal;

	// create toggle for blue/red sample detection. Yellow is also auto allowed
		private boolean isBlueTeam;

	//Declare variables and get motor objects - CS
		private DcMotor motor0;
		private DcMotor motor1;
		private DcMotor motor2;
		private DcMotor motor3;

	// lift, arm, and sClaw1 objects
		private DcMotorEx viper0;
		private DcMotorEx viper1;
		private Servo sElbow0;
		private Servo sClaw1;

	// internal rotation sensing
		private BHI260IMU compass;

	// vars for drive style toggle
		private boolean driveGlobally;
		private boolean prevDriveStyleInput;
		
	// vars for claw input and elbow input
		private boolean prevClawInput;
		private boolean prevElbowInput;
		private boolean elbowDown;
		private boolean clawClosed;
		
	//Var for right stick button input to toggle speed -CS
		private boolean prevRightStickButton;
		
	//Var for a used for bottom pos viper toggle - CC
		private boolean prevA;
		
	//Final vars for autonomous driving in init (calculated in Auto_StarLeft) -CS
		final double COUNTS_PER_CM = 1.17742528;
		
	//var for current bot speed -CC
		private double currSpeed = iv.driveSpeed;
		private double currRotSpeed = iv.rotSpeed;
		

	@Override
	public void init(){
		//Get hardware maps, set directions, set encoders, and set starting positions for all motors and servos -CS
			initDriveMotors();			  
			initLiftMotors();
			initServos();

		// Trying out rotation reading - CC
			compass = hardwareMap.get(BHI260IMU.class, "compass");
			compass.resetYaw();

		// FUN FACT! You can change telemetry font :)
			telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
	}
	

	@Override
	public void loop()
	{
	/*DRIVING: Forward, backward, strafe left, and strafe right all on gamepad1.left_stick
			   Left and right pivot on gamepad1.right_stick -CS	*/
	
		// try out speed limiting with right stick button
			limitSpeed();
		
		// Driving program from BasicDriving_Chase begins  -CS
			double vy = -gamepad1.left_stick_y * currSpeed;				//vy = velocity left
			double vx = gamepad1.left_stick_x * currSpeed;				//vx = veloctiy right
			double w = gamepad1.right_stick_x * currRotSpeed;			//w = rotation velocity

		//Check for change of drive style -CC
			if (gamepad1.start && !prevDriveStyleInput)
				driveGlobally = !driveGlobally;
			prevDriveStyleInput = gamepad1.start;

		// Check for reset of angle -CC
			if (gamepad1.back)
				compass.resetYaw();

		// testing with compass - CC
			double rotation = compass.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

		// setting up driving power vars - CC
			double fl,fr,rl,rr;

		// global direction movement - CC
			if (driveGlobally){
				double rad_rotation = -Math.toRadians(rotation);
				double px = Math.cos(rad_rotation)*vx - Math.sin(rad_rotation)*vy;
				double py = Math.sin(rad_rotation)*vx + Math.cos(rad_rotation)*vy;

				if (Double.isNaN(px))
					px=0;
				if (Double.isNaN(py))
					py=0;

				fl = py + px + w;					//f = forward, r = reverse
				fr = py - px - w;					//r = right, l = left
				rl = py - px + w;
				rr = py + px - w;
			}
				
		// local direction movement (the usual) - CC
			else{
				fl = vy + vx + w;					//f = forward, r = reverse
				fr = vy - vx - w;					//r = right, l = left
				rl = vy - vx + w;
				rr = vy + vx - w;
			}

		// limit power to be <= 1 - CC
			double[] powers = {Math.abs(fl), Math.abs(fr), Math.abs(rl), Math.abs(rr)};
			double max_power = max(powers);
			if (max_power > 1){
				fl /= max_power;
				fr /= max_power;
				rl /= max_power;
				rr /= max_power;
			}

			powerMotors(rl,fl,fr,rr);

	/*VIPER LIFT: Send lift to preset position using buttons & 
				  let driver adjust using triggers -CS */

		/*Get input for viper lift and implement its position limits and set
		  lift to triggers for adjustment after going to preset positions -CS	*/
			float lift = (gamepad2.right_trigger - gamepad2.left_trigger);
			int newTarget = (int)(viper0.getTargetPosition() + lift*25);
			
			if (newTarget < iv.MAX_HEIGHT && newTarget > iv.MIN_HEIGHT){
				 viper0.setTargetPosition(newTarget);
				 viper0.setMode(DcMotor.RunMode.RUN_TO_POSITION);

				 viper1.setTargetPosition(newTarget);
				 viper1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			}
			
		//Get input for sElbow0 and sClaw1 -CS
			if (gamepad2.right_bumper && !prevElbowInput){
				toggleElbow();
			}
			prevElbowInput = gamepad2.right_bumper;
		
			if (gamepad2.left_bumper && !prevClawInput){
				toggleClaw();
			}
			prevClawInput = gamepad2.left_bumper;
				
		/*Turn viper motors off if at or past MIN_POSITION -CS	*/
			if((viper0.getCurrentPosition() <= iv.MIN_HEIGHT || viper1.getCurrentPosition() <= iv.MIN_HEIGHT)){
				viper0.setPower(0);
				viper1.setPower(0);
			}

		//Send viper to preset positions using encoders -CS
			if(gamepad2.a && !prevA){		
				// toggle between home position and slightly above submersible lip -CC
				if (viper0.getTargetPosition() == iv.BOTTOM_PRESET)
					sendLiftTo(iv.VIPER_HOME, iv.altLiftSpeed);
		
				else
					sendLiftTo(iv.BOTTOM_PRESET, iv.altLiftSpeed);
			}
			prevA = gamepad2.a;

			if(gamepad2.x)
				sendLiftTo(iv.MID_PRESET);

			if(gamepad2.y)
				sendLiftTo(iv.TOP_PRESET);
		
			if(gamepad2.b)
				sendLiftTo(iv.VIPER_HOME);


			if (viper0.getTargetPosition() != iv.VIPER_HOME || (viper1.getCurrentPosition() > iv.VIPER_HOME) ||
				viper1.getTargetPosition() != iv.VIPER_HOME || (viper0.getCurrentPosition() > iv.VIPER_HOME)){
				viper0.setPower(iv.liftSpeed);
				viper1.setPower(iv.liftSpeed);	
			}

		//Return telemtry for motor, lift, and compass values
			telemetry(vy, vx, w, currSpeed, currRotSpeed, 
					  fl, fr, rl, rr, lift, iv.liftSpeed, rotation);
	}
	
/*		  **************************************** SUB-METHODS START ************************************************
									(Listed in order of importance to the best of my knowledge,
												feel free to re-arrange - CS)												   */
	
	/*Outputs telemetry data for every variable relating to the driving motors, viper motors, 
	  claw servos, and internal compass -CS
		@param too many for me to bother just read them
		@return void
	*/
	public void telemetry(double vy, double vx, double w, double driveSpeed, double rotSpeed,
						  double fl, double fr, double rl, double rr, 
						  float lift, double liftSpeed, double rotation)
	{
		DecimalFormat df = new DecimalFormat("#,##0.000");

		telemetry.addData("DRIVING INFORMATION", "");
		telemetry.addData("Velocity y", df.format(vy));
		telemetry.addData("Velocity x", df.format(vx));
		telemetry.addData("Rotational velocity", df.format(w));
		telemetry.addData("Drive Speed", driveSpeed);
		telemetry.addData("Rotation Speed", rotSpeed);
		telemetry.addData("", "");

		telemetry.addData("Forward left", df.format(fl));
		telemetry.addData("Forward right", df.format(fr));
		telemetry.addData("Reverse left", df.format(rl));
		telemetry.addData("Reverese right", df.format(rr));
		telemetry.addData("", "");

		telemetry.addData("LIFT INFORMATION", "");
		telemetry.addData("Lift Input", lift);
		telemetry.addData("viper0 Position", viper0.getCurrentPosition());
		telemetry.addData("viper1 Position", viper1.getCurrentPosition());
		telemetry.addData("viper0 Target", viper0.getTargetPosition());
		telemetry.addData("viper1 Target", viper1.getTargetPosition());
		telemetry.addData("Lift speed", liftSpeed);
		telemetry.addData("", "");
		
		telemetry.addData("SERVO INFORMATION", "");		
		telemetry.addData("Elbow Position", sElbow0.getPosition());
		telemetry.addData("Claw Position", sClaw1.getPosition());
		telemetry.addData("", "");
		
		telemetry.addData("Rotation w/ Compass", rotation);
		telemetry.addData("", "");

		telemetry.update();
	}
	

	/*Limits driving speed of motors using the right stick button as a toggle
		@return void
	*/
	public void limitSpeed(){
		if (gamepad1.right_stick_button && !prevRightStickButton){
				if (currSpeed == iv.driveSpeed){
					currSpeed = iv.fineDriveSpeed;
					currRotSpeed = iv.fineRotSpeed;
				}
				else{
					currSpeed = iv.driveSpeed;
					currRotSpeed = iv.rotSpeed;
				}
			}
		prevRightStickButton = gamepad1.right_stick_button;
	}
	
	
	private boolean claw_closed;
	/*Allows you to toggle claw button in stead of holding it down -CS
		@return void
	*/
	public void toggleClaw(){
		//sClaw1.setPosition(sClaw1.getPosition()+.02);		//Testing claw pos by incrementing it +.02 w/ each input -CS
		if (claw_closed) {
			sClaw1.setPosition(iv.CLAW_OPEN);
			claw_closed = false;
		} else {
			sClaw1.setPosition(iv.CLAW_CLOSE);
			claw_closed = true;
		}
	}
	
	private boolean elbow_down;
	/*Allows you to toggle elbow button in stead of holding it down -CS
		@return void
	*/
	public void toggleElbow(){
		//sElbow0.setPosition(sElbow0.getPosition() + 0.02);		//Testing elbow pos by incrementing it +.02 w/ each input -CS
		if (elbow_down){
			sElbow0.setPosition(iv.ELBOW_UP);
			elbow_down = false;
		} else {
			sElbow0.setPosition(iv.ELBOW_GRAB);
			elbow_down = true;
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


	/*Gets the motors from the control hub, sets their direction and their brake behavior.
	  Directions are set so that when all power values are positive, all motors move the
	  robot forwards. -CS
			@return void
	*/
	public void initDriveMotors(){
		motor0 = hardwareMap.get(DcMotor.class, "motor0");
		motor1 = hardwareMap.get(DcMotor.class, "motor1");
		motor2 = hardwareMap.get(DcMotor.class, "motor2");
		motor3 = hardwareMap.get(DcMotor.class, "motor3");

		motor0.setDirection(DcMotorSimple.Direction.REVERSE);
		motor1.setDirection(DcMotorSimple.Direction.FORWARD);
		motor2.setDirection(DcMotorSimple.Direction.FORWARD);
		motor3.setDirection(DcMotorSimple.Direction.REVERSE);
		
		motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}
	
	
	public void initLiftMotors(){
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
	
	
	/*Gets the elbow and claw servo from the control hub and sets their directions.
	  Sends both servos to their initialization positions -CS
			@return void
	*/
	public void initServos(){
		sElbow0 = hardwareMap.get(Servo.class, "sElbow0");
		sElbow0.setDirection(Servo.Direction.REVERSE);
		sElbow0.setPosition(iv.ELBOW_UP);
			
		sClaw1 = hardwareMap.get(Servo.class, "sClaw1");
		sClaw1.setPosition(iv.CLAW_OPEN);
	}
	
	
	/*Power all motors with specific speeds -CS
		@param rl, fl, fr, rr - the specific speeds for each respective motor
		@return void
	*/
	public void powerMotors(double rl, double fl, double fr, double rr){
	  motor0.setPower(fl);
	  motor1.setPower(fr);
	  motor2.setPower(rr);
	  motor3.setPower(rl);
	}


	/*Ensures each motor does not exceed the max speed
		@param powers[] - the max speed for each motor
		@return ret	 - the largest value in powers array
	*/
	public double max(double[] powers){
		double ret = powers[0];

		for (int i=0; i<powers.length; i++){
			if (powers[i] > ret){
				ret = powers[i];
			}
		}

		return ret;
	}
}