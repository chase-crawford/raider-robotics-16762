package	org.firstinspires.ftc.teamcode;

import	com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import	com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import	com.qualcomm.robotcore.eventloop.opmode.OpMode;
import	com.qualcomm.robotcore.hardware.DcMotor;
import	com.qualcomm.robotcore.hardware.Servo;
import	com.qualcomm.robotcore.hardware.DcMotorSimple;
import	java.text.DecimalFormat;
import	com.qualcomm.robotcore.hardware.Servo;
import	java.text.DecimalFormat;
import	com.qualcomm.robotcore.eventloop.opmode.OpMode;
import	com.qualcomm.robotcore.hardware.DcMotorSimple;
import	com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import	com.qualcomm.robotcore.hardware.DcMotor;

//@TeleOp(name="BasicDriving_Chase")

public	class	WheelEncoderTesting_Chase	extends	OpMode{

	private	DcMotor	m0;
	private	DcMotor	m1;
	private	DcMotor	m2;
	private	DcMotor	m3;
	private DcMotor mLift;
	private Servo claw;
	private DcMotor mArm;

	@Override
	public void init(){
					
					m0 = hardwareMap.get(DcMotor.class, "Motor0");
					m1 = hardwareMap.get(DcMotor.class, "Motor1");
					m2 = hardwareMap.get(DcMotor.class, "Motor2");
					m3 = hardwareMap.get(DcMotor.class, "Motor3");

	//Right	side motors	forward, Left reverse
		m0.setDirection(DcMotorSimple.Direction.REVERSE);
		m1.setDirection(DcMotorSimple.Direction.REVERSE);
		m2.setDirection(DcMotorSimple.Direction.FORWARD);
		m3.setDirection(DcMotorSimple.Direction.FORWARD);

		m0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		mLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);			
		m0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		mLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			
		m0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			
		mLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		mArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			
	}

					@Override
					public	void	loop(){
									//	get	gamepad	input	for	movement
													double	vy	=	gamepad1.left_stick_y*-1;							//vy	=	velocity	left
													double	vx	=	gamepad1.left_stick_x;										//vx	=	veloctiy	right
													double	w	=	gamepad1.right_stick_x;										//w	=	rotation	velocity

									//	calculate	movement	motor	powers
													double	fl	=	vy	+	vx	+	w;												//f	=	forward,	r	=	reverse
													double	fr	=	vy	-	vx	-	w;												//r	=	right,	l	=	left
													double	rl	=	vy	-	vx	+	w;
													double	rr	=	vy	+	vx	-	w;

									//	make	array	of	power	m
													double[]	powers	=	{Math.abs(fl),	Math.abs(fr),	Math.abs(rl),	Math.abs(rr)};
									
									//	get	max	power
													double	max_power	=	max(powers);
													
									//	set	powers	proportional	to	limit	max	power	to	1
													if	(max_power	>	1){
																	fl	/=	max_power;
																	fr	/=	max_power;
																	rl	/=	max_power;
																	rr	/=	max_power;
													}
					
					//output	telemetry				
									telemetry(vy,	vx,	w,	fl,	fr,	rl,	rr);	

						//	set	power	of	movement	motors
									powerMotors(rl,fl,fr,rr);
									
									
					//	LIFT	SYSTEM
									//	get	input	for	viper	lift
													float	lift	=	(gamepad1.right_trigger	-	gamepad1.left_trigger);
													mLift.setTargetPosition((int)(mLift.getTargetPosition()	+	lift*25));
													mLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
													mLift.setPower(.25);
													
									//	get	input	for	arm	rotation
													float	arm	=	gamepad1.dpad_up	?	-.25f	:	gamepad1.dpad_down	?	.25f	:	0;
													mArm.setPower(arm);
													
									//	get	input	for	servo
													if	(gamepad1.x)	claw.setPosition(claw.getPosition()	-	.1f);
													if	(gamepad1.b)	claw.setPosition(claw.getPosition()	+	.1f);
													
									telemetry.addData("Lift	Input",	lift);
									telemetry.addData("Arm	Input",	arm);
									telemetry.addData("Servo	Position",	claw.getPosition());
									telemetry.addData("Lift	Position",	mLift.getCurrentPosition());
									telemetry.addData("Lift	Target",	mLift.getTargetPosition());
													
							}

	public	void	powerMotors(double	speed){
			m0.setPower(speed);
			m1.setPower(speed);
			m2.setPower(speed);
			m3.setPower(speed);
	}

	public	void	powerMotors(double	zero,	double	one,	double	two,	double	three){
			m0.setPower(zero);
			m1.setPower(one);
			m2.setPower(two);
			m3.setPower(three);
	}

	public	double	max(double[]	powers){
					double	ret	=	powers[0];

					for	(int	i=0;	i<powers.length;	i++){
									if	(powers[i]	>	ret){
													ret	=	powers[i];
									}
					}

					return	ret;
	}
	
	//This	submethod	will
	public	void	telemetry(double	vy,	double	vx,	double	w,	double	fl,	double	fr,
																							double	rl,	double	rr)
	{
					DecimalFormat	df	=	new	DecimalFormat("#,##0.000");
					
					telemetry.addData("Velocity	y",	df.format(vy));
					telemetry.addData("Velocity	x",	df.format(vx));
					telemetry.addData("Rotational	velocity",	df.format(w));
					telemetry.addData("",	"");
					
					telemetry.addData("Forward	left",	df.format(fl));
					telemetry.addData("Forward	right",	df.format(fr));
					telemetry.addData("Reverse	left",	df.format(rl));
					telemetry.addData("Reverese	right",	df.format(rr));
	}

}
