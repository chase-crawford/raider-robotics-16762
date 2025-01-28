// package org.firstinspires.ftc.teamcode;
// 
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.hardware.DcMotorSimple;
// 
// @Autonomous(name="Autonomous - Kendall")
// 
// public class AutonomousKJC extends LinearKJC {
//	  // center: 0 range -2 to 2
//	  public static double intersect(double x) {
//			return x / 3.0;
//	  }
//	  
//	  @Override
//	  public void runAuto() {
//			 // Wait till we can begin
//			 while (!opModeIsActive()) {}
//			 
//			////							  BL  FR  BR  FR LIFT ARM CLAW
//			//hardware.runToVector(telemetry,  1,  1, -1, -1,  0,  0,  0);
//			//hardware.runToVector(telemetry,  0,  0,  0,  0,  3,  0,  0);
//			//hardware.runToVector(telemetry,  0,  0,  0,  0, -3,  0,  0);	   
//			//hardware.runToVector(telemetry,  1,  1,  1,  1,  0,  0,  0);	   
//			//hardware.runToVector(telemetry, -1, -1, -1, -1,  0,  0,  0); 
//			
//			goToPosition(intersect(-2), intersect(-2));
//			goToPosition(intersect(-2), intersect(2));
//			goToPosition(intersect(2), intersect(2));
//			goToPosition(intersect(2), intersect(-2));
//			
//			goToPosition(intersect(-1.5), intersect(-2));
//			goToPosition(intersect(-2), intersect(-2));
//	  }
// }
// 
