package org.firstinspires.ftc.teamcode.Year24_25.OldOrTesting;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.io.File;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;


/**
 * Created by Sarthak on 6/1/2019. Updated by Catherine Shea starting 11/5/24
 */

public class OdometryGlobalCoordinatePosition implements Runnable{
	//Odometry wheels
		private DcMotor odoY_Left, odoY_Right, odoX;

	//Thead run condition
		private boolean isRunning = true;

	//Position variables used for storage and calculations
		double odoY_RightPosition, odoY_LeftPosition, odoX_Position = 0;
		double changeInRobotOrientation;
		private double robotGlobalXCoordinatePosition, robotGlobalYCoordinatePosition;
		private double robotOrientationRadians;
		private double odoY_LeftWheelPosition, odoY_RightWheelPosition, odoX_WheelPosition;
		private double previousOdoY_LeftWheelPosition, previousOdoY_RightWheelPosition; 
		private double prevNormalEncoderWheelPosition; 

	//Algorithm constants
		private double robotEncoderWheelDistance;
		private double odoXTickPerDegreeOffset;

	//Sleep time interval (milliseconds) for the position update thread
		private int sleepTime;

	//Files to access the algorithm constants
		private File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
		private File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

		private int odoY_LeftPositionMultiplier = 1;
		private int odoY_RightPositionMultiplier = 1;
		private int odoX_PositionMultiplier = 1;

	/*This method is a constructor for GlobalCoordinatePosition Thread
		@param verticalEncoderLeft left odometry encoder, facing the vertical direction
		@param verticalEncoderRight right odometry encoder, facing the vertical direction
		@param odoX horizontal odometry encoder, perpendicular to the other two odometry encoder wheels
		@param threadSleepDelay delay in milliseconds for the GlobalPositionUpdate thread (50-75 milliseconds is suggested)
	*/
	public OdometryGlobalCoordinatePosition(DcMotor odoY_Left, DcMotor odoY_Right, DcMotor odoX, double COUNTS_PER_INCH, int threadSleepDelay){
		this.odoY_Left = odoY_Left;
		this.odoY_Right = odoY_Right;
		this.odoX = odoX;
		sleepTime = threadSleepDelay;

		robotEncoderWheelDistance = Double.parseDouble(ReadWriteFile.readFile(wheelBaseSeparationFile).trim()) * COUNTS_PER_INCH;
		this.odoXTickPerDegreeOffset = Double.parseDouble(ReadWriteFile.readFile(horizontalTickOffsetFile).trim());
	}

	/*This method updates the global (x, y, theta) coordinate position of 
	  the robot using the odometry encoders
		@return void
	*/  
	private void globalCoordinatePositionUpdate(){
		//Get Current Positions
			odoY_LeftPosition = (odoY_Left.getCurrentPosition() * odoY_LeftPositionMultiplier);
			odoY_RightPosition = (odoY_Right.getCurrentPosition() * odoY_RightPositionMultiplier);
	
			double leftChange = odoY_LeftPosition - previousOdoY_LeftWheelPosition;
			double rightChange = odoY_RightPosition - previousOdoY_RightWheelPosition;

		//Calculate Angle
			changeInRobotOrientation = (leftChange - rightChange) / (robotEncoderWheelDistance);
			robotOrientationRadians = ((robotOrientationRadians + changeInRobotOrientation));

		//Get the components of the motion
			double normalEncoderWheelPosition = (odoY_Left.getCurrentPosition()*odoY_LeftPositionMultiplier);
			double rawHorizontalChange = normalEncoderWheelPosition - prevNormalEncoderWheelPosition;
			double horizontalChange = rawHorizontalChange - (changeInRobotOrientation*odoXTickPerDegreeOffset);
	
			double p = ((rightChange + leftChange) / 2);
			double n = horizontalChange;

		//Calculate and update the position values
			robotGlobalXCoordinatePosition = robotGlobalXCoordinatePosition + (p*Math.sin(robotOrientationRadians) + n*Math.cos(robotOrientationRadians));
			robotGlobalYCoordinatePosition = robotGlobalYCoordinatePosition + (p*Math.cos(robotOrientationRadians) - n*Math.sin(robotOrientationRadians));
	
			previousOdoY_LeftWheelPosition = odoY_LeftWheelPosition;
			previousOdoY_RightWheelPosition = odoY_RightWheelPosition;
			prevNormalEncoderWheelPosition = normalEncoderWheelPosition;
	}

	/*Returns the robot's global x coordinate
		@return global x coordinate
	*/
	public double returnXCoordinate(){
		return robotGlobalXCoordinatePosition; }

	/*Returns the robot's global y coordinate
		@return global y coordinate
	*/
	public double returnYCoordinate(){
		return robotGlobalYCoordinatePosition; }

	/*Returns the robot's global orientation
		@return global orientation, in degrees
	*/
	public double returnOrientation(){ return Math.toDegrees(robotOrientationRadians) % 360; }

	/*Stops the position update thread
		@return void
	*/
	public void stop(){ isRunning = false; }

	public void reverseLeftEncoder(){
		if(odoY_LeftPositionMultiplier == 1){
			odoY_LeftPositionMultiplier = -1;
		}else{
			odoY_LeftPositionMultiplier = 1;
		}
	}

	public void reverseRightEncoder(){
		if(odoY_RightPositionMultiplier == 1){
			odoY_RightPositionMultiplier = -1;
		}else{
			odoY_RightPositionMultiplier = 1;
		}
	}

	public void reverseNormalEncoder(){
		if(odoY_LeftPositionMultiplier == 1){
			odoY_LeftPositionMultiplier = -1;
		}else{
			odoY_LeftPositionMultiplier = 1;
		}
	}

	/*Runs the thread
		@return void
	*/
	@Override
	public void run() 
	{
		while(isRunning) 
		{
			globalCoordinatePositionUpdate();
			try {
				Thread.sleep(sleepTime);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}
	
}

