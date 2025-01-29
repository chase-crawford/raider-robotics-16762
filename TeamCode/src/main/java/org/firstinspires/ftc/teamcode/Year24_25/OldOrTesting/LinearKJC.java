// package org.firstinspires.ftc.teamcode;
// 
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.hardware.HardwareMap;
// import org.firstinspires.ftc.robotcore.external.Telemetry;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// 
// public abstract class LinearKJC extends LinearOpMode {
//	   public DcMotor motors[]; // 4/5: Lift
//	   public Servo servos[]; // 0: Claw
//	   public DcMotor motor0; // Alias for motors[0]
//	   public DcMotor motor1; // Alias for motors[1]
//	   public DcMotor motor2; // Alias for motors[2]
//	   public DcMotor motor3; // Alias for motors[3]
//	   public static final int MAX_LIFT_TICKS = 4200;
//	   private AprilTagController april;
//	  
//	   public abstract void runAuto();
//	   
//	   @Override
//	   public void runOpMode() {
//			 motors = new DcMotor[6];
//			 motors[0] = hardwareMap.get(DcMotor.class, "motor0");
//			 motors[1] = hardwareMap.get(DcMotor.class, "motor1");
//			 motors[2] = hardwareMap.get(DcMotor.class, "motor2");
//			 motors[3] = hardwareMap.get(DcMotor.class, "motor3");
//			 motors[4] = hardwareMap.get(DcMotor.class, "viper0");
//			 motors[5] = hardwareMap.get(DcMotor.class, "viper1"); // TODO
//			 
//			 servos = new Servo[0];
//			 //servos[0] = hardwareMap.get(Servo.class, "Servo"); // TODO
//			 
//			 motor0 = motors[0];
//			 motor1 = motors[1];
//			 motor2 = motors[2];
//			 motor3 = motors[3];
//			 
//			 for (int i = 0; i < motors.length; i++) {
//				   motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//				   motors[i].setDirection(DcMotorSimple.Direction.FORWARD);
//			 }
//			 
//			 motors[4].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//			 motors[5].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//			 
//			 april = new AprilTagController(hardwareMap.get(WebcamName.class, "Webcam"));
//			 runAuto();
//	   }
//	   
//	   public void liftTo(int ticks) {
//			 if (ticks > MAX_LIFT_TICKS) {
//				   int fail = 0/0; // crash
//				   return; // just incase
//			 }
//			 
//			 motors[4].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//			 motors[5].setMode(DcMotor.RunMode.RUN_TO_POSITION);
//			 
//			 motors[4].setTargetPosition(ticks);
//			 motors[5].setTargetPosition(ticks);
//			 
//			 motors[4].setPower(0.2);
//			 motors[5].setPower(0.2);
//			 
//			 while (Math.abs(motors[4].getCurrentPosition() - ticks) > 4); // Maybe 5 also
// 
//			 motors[4].setPower(0);
//			 motors[5].setPower(0);
//			 
//			 motors[4].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//			 motors[5].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//	   }
//	
//	   public void goToPosition(double destx, double desty) {
//			 for (int i = 0; i < motors.length; i++) {
//				   motors[i].setDirection(DcMotorSimple.Direction.FORWARD);
//				   motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//				   motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//			 }
//			 
//			 double bobx, boby, bobr; // bobr is in radians [-180 d, 180 d]
//			 bobx = boby = bobr = Double.NaN; // uninitialized
//			 while (opModeIsActive()) {
//				   telemetry.addData("destination", "x: %f y: %f", destx, desty);
//				   
//		   // Update stored position to detected April Tag's if possible
//				   april.update();
//				   if (april.seesTag) {
//						 bobx = april.x;
//						 boby = april.y;
//						 double rot = april.rotation - 180;
//						 rot += 90; if (rot > 180) rot = 360 - rot;
//						 telemetry.addData("rotation translation",
//										   "april: %f real: %f", april.rotation, rot);
//						 bobr = Math.toRadians(rot);
//				   } else if (bobr != bobr) { // NaN
//						 telemetry.addData("goToPosition", "waiting for camera");
//						 telemetry.update();
//						 continue;
//				   }
//				   
//		   // Check if we are there yet
//				   double dist = Math.sqrt(
//						 Math.pow(desty - boby, 2) +
//						 Math.pow(destx - bobx, 2));
//				   if (dist < 0.005) break;
//		   
//		   telemetry.addData("position", "bobx: %f boby: %f bobr: %f dist: %f",
//						 bobx, boby, Math.toDegrees(bobr), dist);
//				   
//		   // Translate and rotate the dest so that Bob is the origin
//		   // The rotation assumes (0, 0) to (1, 1) is 45*, if it negative just flip
//		   // the signs in the calculation of px and py.
//		   double ux = destx - bobx, uy = desty - boby; // [u]nrotated with origin
//		   double px = ux * Math.cos(bobr) + uy * Math.sin(bobr);
//		   double py = uy * Math.cos(bobr) - ux * Math.sin(bobr);
//				   
//		   telemetry.addData("vectors", "px: %f py: %f", px, py);
// 
//		   // Calculate motor powers
//				   double fl = py + px;
//				   double fr = py - px;
//				   double bl = py - px;
//				   double br = py + px;
//				   double scale = Math.max(
//						 Math.max(Math.abs(fl), Math.abs(fr)),
//						 Math.max(Math.abs(bl), Math.abs(br)));
//				   fl /= scale; fr /= scale; bl /= scale; br /= scale;
//				   
//				   telemetry.addData("powers", "scale: %f, fl: %f fr: %f bl: %f br: %f",
//						 scale, fl, fr, bl, br);
// 
//		   // Run the motors
//				   motors[0].setPower(0.3 * fl);
//				   motors[1].setPower(0.3 * fr);
//				   motors[2].setPower(0.3 * br);
//				   motors[3].setPower(0.3 * bl);
//				   
//				   telemetry.update();
//			 }
//			 
//			 motors[0].setPower(0);
//			 motors[1].setPower(0);
//			 motors[2].setPower(0);
//			 motors[3].setPower(0);
//	   }
// }
//	   
// 
