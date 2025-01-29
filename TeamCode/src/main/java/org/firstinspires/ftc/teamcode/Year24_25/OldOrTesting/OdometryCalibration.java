package org.firstinspires.ftc.teamcode.Year24_25.OldOrTesting;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import java.io.File;
//import android.os.Environment;


/**
	  Created by Sarthak on 6/1/2019.
	  Edited by Catherine Shea starting 10/11/2024

	  Odometry system calibration. Run this OpMode to generate the necessary constants to calculate the robot's
	 global position on the field.
	  The Global Positioning Algorithm will not function and will throw an error if this program is not run first
*/


//@TeleOp(name = "Odometry System Calibration", group = "Calibration")
public class OdometryCalibration extends OpMode
{
	//Drive motors
		DcMotor motor1, motor2, motor0, motor3;	//right_front, right_back, left_front, left_back

   //Odometry Wheels
		DcMotor odoY_Left, odoY_Right, odoX;

   //IMU Sensor
		BNO055IMU imu;

   //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
		static String rfName = "motor1", rbName = "motor2", lfName = "motor0", lbName = "motor1";
		// odomentry encoders plug into the same ports as motors
		static String yL_Encoder = rbName, rR_Encoder = lfName, h_Encoder = rfName;

		final double PIVOT_SPEED = 0.5;

	//The amount of encoder ticks for each inch the robot moves. THIS WILL CHANGE FOR EACH ROBOT AND NEEDS TO BE UPDATED HERE
	//																					- Has been updated as of 10/18/24
		final double COUNTS_PER_MOTOR_REV	= 384.5;
		final double DRIVE_GEAR_REDUCTION	= 13.7;
		final double WHEEL_DIAMETER_INCHES	= 5.51181;

		final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

		ElapsedTime timer = new ElapsedTime();
		double horizontalTickOffset = 0;

   //Text files to write the values to. The files are stored in the robot controller under Internal Storage\FIRST\settings
			//App.Util is from when phones were used - find alternate class
	// Cady.. I think we need to switch to Android Studio. I found an example that uses an Environment class,
	// but it is only available in Android Studio. No clue otherwise on how to get into it. - CC
		//File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
		//File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");
		
		// String file = Environment.getDataDirectory().getAbsolutePath();
		// their code says Environment.getExternalStorageDirectory().getAbsolutePath(); but idk - CC

	public void init()
	{
		//Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
			initHardwareMap(rfName, rbName, lfName, lbName, rbName, lfName, rfName); //yL_Encoder, yR_Encoder, h_Encoder);

		//Initialize IMU hardware map value. PLEASE UPDATE THIS VALUE TO MATCH YOUR CONFIGURATION
			imu = hardwareMap.get(BNO055IMU.class, "imu");

		//Initialize IMU parameters
			BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
			parameters.angleUnit			= BNO055IMU.AngleUnit.DEGREES;
			parameters.accelUnit			= BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
			parameters.calibrationDataFile  = "BNO055IMUCalibration.json"; // see the calibration sample opmode
			parameters.loggingEnabled	   = true;
			parameters.loggingTag		   = "IMU";
			parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

			imu.initialize(parameters);
			telemetry.addData("Odometry System Calibration Status", "IMU Init Complete");
			telemetry.clear();

		//Odometry System Calibration Init Complete
			telemetry.addData("Odometry System Calibration Status", "Init Complete");
			telemetry.update();
	}

   @Override
   public void loop(){ //throws InterruptedException {
		//Begin calibration (if robot is unable to pivot at these speeds, please adjust the constant at the top of the code
			while(getZAngle() < 90){
				motor1.setPower(-PIVOT_SPEED);
				motor2.setPower(-PIVOT_SPEED);
				motor0.setPower(PIVOT_SPEED);
				motor3.setPower(PIVOT_SPEED);

				if(getZAngle() < 60)
					setPowerAll(-PIVOT_SPEED, -PIVOT_SPEED, PIVOT_SPEED, PIVOT_SPEED);

				else
					setPowerAll(-PIVOT_SPEED/2, -PIVOT_SPEED/2, PIVOT_SPEED/2, PIVOT_SPEED/2);

				telemetry.addData("IMU Angle", getZAngle());
			telemetry.update();
			}

		//Stop the robot
			setPowerAll(0, 0, 0, 0);
			timer.reset();
			while(timer.milliseconds() < 1000){
			 telemetry.addData("IMU Angle", getZAngle());
			telemetry.update();
			}

		//Record IMU and encoder values to calculate the constants for the global position algorithm
			double angle = getZAngle();

	  /*
		Encoder Difference is calculated by the formula (leftEncoder - rightEncoder)
		Since the left encoder is also mapped to a drive motor, the encoder value needs to be reversed with the
		negative sign in front

		THIS MAY NEED TO BE CHANGED FOR EACH ROBOT
		*/
		double encoderDifference = -(Math.abs(odoY_Left.getCurrentPosition())) + (Math.abs(odoY_Right.getCurrentPosition()));

		double verticalEncoderTickOffsetPerDegree = encoderDifference/angle;

		double wheelBaseSeparation = (2*90*verticalEncoderTickOffsetPerDegree)/(Math.PI*COUNTS_PER_INCH);

		horizontalTickOffset = odoX.getCurrentPosition()/Math.toRadians(getZAngle());

		//Write the constants to text files
			  //ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
			  //ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffset));

			//Update telemetry output
				telemetry(wheelBaseSeparation, horizontalTickOffset, getZAngle(), odoY_Left.getCurrentPosition(),
							 odoY_Right.getCurrentPosition(), odoX.getCurrentPosition(), verticalEncoderTickOffsetPerDegree);
	}


	private void initHardwareMap(String rfName, String rbName, String lfName, String lbName,
								 String vlEncoderName, String vrEncoderName, String hEncoderName)
	{
		motor1 = hardwareMap.get(DcMotor.class, rfName);				//Compare with object names on lines 37 and 38
		motor2 = hardwareMap.get(DcMotor.class, rbName);	
		motor0 = hardwareMap.get(DcMotor.class, lfName);	
		motor3 = hardwareMap.get(DcMotor.class, lbName);	

		odoY_Left = hardwareMap.dcMotor.get(vlEncoderName);
		odoY_Right = hardwareMap.dcMotor.get(vrEncoderName);
		odoX = hardwareMap.dcMotor.get(hEncoderName);

		motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		odoY_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		odoY_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		odoX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		odoY_Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		odoY_Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		odoX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


		motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		motor0.setDirection(DcMotorSimple.Direction.REVERSE);
		motor1.setDirection(DcMotorSimple.Direction.REVERSE);
		motor2.setDirection(DcMotorSimple.Direction.REVERSE);


		telemetry.addData("Status", "Hardware Map Init Complete");
		telemetry.update();
	}


	/**
	 * Gets the orientation of the robot using the REV IMU
	 *	 @return the angle of the robot
	 */
	private double getZAngle(){
		return (-imu.getAngularOrientation().firstAngle);	//https://javadoc.io/static/org.firstinspires.ftc/RobotCore/10.1.0/org/firstinspires/ftc/robotcore/external/navigation/Orientation.html
	}


	/**
	  Sets power to all four drive motors
		  @param rf power for right front motor
		  @param rb power for right back motor
		  @param lf power for left front motor
		  @param lb power for left back motor
	 */
	private void setPowerAll(double rf, double rb, double lf, double lb){
		motor1.setPower(rf);
		motor2.setPower(rb);
		motor0.setPower(lf);
		motor3.setPower(lb);
	}


	public void telemetry(double wheelBaseSeparation, double horizontalTickOffset, double getZAngle, double verticalLeftPos,
								  double verticalRightPos, double horizontalPos, double verticalEncoderTickOffsetPerDegree)
	{
		telemetry.addData("Odometry System Calibration Status", "Calibration Complete");

		//Display calculated constants
			telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
			telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);

		//Display raw values
			telemetry.addData("IMU Angle", getZAngle());
			telemetry.addData("Vertical Left Position", -odoY_Left.getCurrentPosition());
			telemetry.addData("Vertical Right Position", odoY_Right.getCurrentPosition());
			telemetry.addData("Horizontal Position", odoX.getCurrentPosition());
			telemetry.addData("Vertical Encoder Offset", verticalEncoderTickOffsetPerDegree);

		//Update values
			telemetry.update();
	 }

}
