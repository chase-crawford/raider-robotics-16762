package org.firstinspires.ftc;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Hardware Testing")

public class HardwareTesting extends OpMode {
		//Declare variables and get motor objects
				private DcMotor motors[];
				private DcMotor motor0; // Alias for motors[0]
				private DcMotor motor1; // Alias for motors[1]
				private DcMotor motor2; // Alias for motors[2]
				private DcMotor motor3; // Alias for motors[03]
				private Servo servo0;
				private CRServo crservo0;

		@Override
		public void init() {
				//Get hardware map for objects
						motors = new DcMotor[4];
						motors[0] = hardwareMap.get(DcMotor.class, "Motor0");
						motors[1] = hardwareMap.get(DcMotor.class, "Motor1");
						motors[2] = hardwareMap.get(DcMotor.class, "Motor2");
						motors[3] = hardwareMap.get(DcMotor.class, "Motor3");
						motor0 = motors[0];
						motor1 = motors[1];
						motor2 = motors[2];
						motor3 = motors[3];
						
						motors[0].setDirection(DcMotorSimple.Direction.FORWARD);
						motors[1].setDirection(DcMotorSimple.Direction.FORWARD);
						motors[2].setDirection(DcMotorSimple.Direction.FORWARD);
						motors[3].setDirection(DcMotorSimple.Direction.FORWARD);
						
						motors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
						motors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
						motors[2].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
						motors[3].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
						
						servo0 = hardwareMap.get(Servo.class, "Servo0");
						servo0.setDirection(Servo.Direction.FORWARD);
						
						crservo0 = hardwareMap.get(CRServo.class, "CrServo0");
		}
		
		// KJC
		public void waitForPosition(DcMotor motor, int position) {
				while (motor.getCurrentPosition() != position) {}
		}
		
		// KJC
		public void runToVector(int targets[]) {
				for (int i = 0; i < motors.length; i++) {
						motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
						motors[i].setPower(0.5);
						motors[i].setTargetPosition(targets[i]);
						motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
				}
				
				// TODO: async checking so zero power can be set faster
				for (int i = 0; i < motors.length; i++) {
						waitForPosition(motors[i], targets[i]);
				}
				
				for (int i = 0; i < motors.length; i++) {
						motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				}
		}
		
		@Override
		public void loop() {
				//Bind hardware testing to controller inputs
				// If A is selected then motor 0 will spin backwards
				// otherwise backwards
				// Same with B and motor 1
						motors[0].setPower(gamepad1.a ? -0.5 : 0.5);
						motors[1].setPower(gamepad1.b ? -0.5 : 0.5);
						servo0.setPosition(gamepad1.x ? 0 : 1);
						crservo0.getController().setServoPosition(crservo0.getPortNumber(), gamepad1.y ? 0 : 1);
						
				//Autonomous testing
						if(gamepad1.right_bumper){
								// KJC
								int pos1[] = {400, 400, -400, -400};
								int pos2[] = {-400, -400, 400, 400};
								int pos3[] = {400, 400, 0, 0};
								int pos4[] = {0, 0, 400, 400};
								runToVector(pos1);
								runToVector(pos2);
								runToVector(pos3);
								runToVector(pos4);
						}
						
		}
}
