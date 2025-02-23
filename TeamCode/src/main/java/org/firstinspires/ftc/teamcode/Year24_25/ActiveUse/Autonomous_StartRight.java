package org.firstinspires.ftc.teamcode.Year24_25.ActiveUse;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


//@Autonomous(name = "Autonomous_StartRight")
public class Autonomous_StartRight extends LinearOpMode{
	//Create object for InitVars class -CS
		InitVars IV = new InitVars();

	//Create object for AutoMethods class
		AutoMethods AM;
		

	//Measurements ( here to keep Wilson's work, not used in code - CC )
		static final double	COUNTS_PER_MOTOR_REV = 1440;		// eg: TETRIX Motor Encoder
		static final double	DRIVE_GEAR_REDUCTION = 1.0; 		// No External Gearing.
		static final double	WHEEL_DIAMETER_CM	 = 104.0;		// For figuring circumference
		static final double	COUNTS_PER_CM		 = 1.17742528;
					  
	@Override
	public void runOpMode()
	{
		AM = new AutoMethods(hardwareMap, telemetry, this);
		AM.init();
		
		//Begin driving program -CS
		/*	STEPS FOR AUTONOMOUS PATH
				1. keep elbow in upright position, close claw on specimen -CC
		
				WAIT FOR START
				
				2. drive forward to be in front of submersible
				3. drive right to be in line with the bar 
				
				4. wait for lift to get to high bar position 
				5. move lift down to snap specimen to bar 
				
				6. wait for lift to move down 
				7. let specimen go 
				
				8. bring lift down before continuing to drive 
				9. move robot to right of submersible 
				10. move forward to be right of alliance samples - CC
				
				11. drive to be infront of leftmost sample
				12. push left sample into observation zone
				13. drive back behind samples
				14. push middle sample to net zone
				
				15. drive backwards to middle of field
				16. drive left to get in ascent zone
		*/
			
		//AM.raiseElbow();
		//AM.openClaw();
		AM.startWithSpecimen();
			
		waitForStart();
		
		hangHeldSpecimen();
		//hangHumanSpecimens(5);
		
		//comment :D
		pushFirstSample();
	
		//comment :D
		pushOtherSamplesAndPark();
	}
	
	
	// This method will move the robot from the starting position to the
	// specimen rung hanging area and attach a blue specimen to the highest rung.
	// PRECONDITION: Robot is holding a specimen
	public void hangHeldSpecimen(){
		AM.raiseElbow();
		AM.sendLiftTo(1470);					
		AM.driveTicks(760);
		
		AM.sendLiftTo(1470-700);
		AM.waitForLift();
		AM.openClaw();	
		
		AM.sendLiftTo(IV.VIPER_HOME);			
		//AM.waitForLift();
	}
	
	/* This method will move the robot to the observation zone and pick up a human-placed specimen.
	It will then hang the specimen on the high rung and repeat this process n times 
	PRECONDITION: Robot must first complete hangHeldSpecimen. */
		private void hangHumanSpecimens(int n){
			final int stepDelta = 75;
			int currDelta = 0;
			
			for (int i=0; i<n; i++){
				// drive to specimen pickup
					AM.sendLiftTo(IV.BOTTOM_PRESET);
					AM.driveTicks(-635);
					AM.driveTicks(0, 0, 800);
					AM.lowerElbow();
					AM.correctRotation(-90);
					AM.driveTicks(740+currDelta);
					
				// in order to not put specimens on specimens, change the horizontal pos
				// each iteration
					currDelta += stepDelta;
				
				// lower elbow to grab, correct rot to allow time for elbow
				// and close claw to grab
					AM.sendLiftTo(250);
					AM.waitForLift();
					AM.closeClaw();
					AM.driveTicks(-25);
					AM.sendLiftTo(400);						//IV.BOTTOM_PRESET + 100
					
				// drive back to rungs
					AM.driveTicks(-200);
					AM.driveTicks(-725, -700);				//-725, -700
					AM.raiseElbow();
					
					AM.sendLiftTo(1470);
					AM.driveTicks(0, 0, -800);
					AM.correctRotation();
					AM.driveTicks(-currDelta, 0);
				
				// raise elbow, move lift to rung pos, and wait for that to be done
					AM.waitForLift();
				
				// snap specimen on rung and open claw to let go
					AM.sendLiftTo(1470-600);
					AM.waitForLift();
					AM.openClaw();
			}
		}
	
	
	public void pushFirstSample(){
		AM.driveTicks(1100, 0);				//move ->
		AM.correctRotation();
		
		AM.driveCM(800, false);				//go up to the side of sample			
		AM.driveCM(400, 0);					//move -> to be infront of first sample
		AM.correctRotation();
		
		AM.driveCM(-1300, true);			//move back to put sample in obsv. zone @ slower speed
	}
	
	
	public void pushOtherSamplesAndPark(){
		AM.correctRotation();				//go back up to push second sample
		AM.driveCM(1300, false);
		AM.driveCM(300, 0);
		AM.driveCM(-1300, true);
		
		/*AM.correctRotation();				//go back up to push third sample
		AM.driveCM(1200, false);			//TOO CLOSE TO ARENA WALL TO USE
		AM.driveCM(100, 0);
		AM.driveCM(-1200, true);*/
		
		AM.correctRotation();				//go back up to park
		AM.driveCM(1300, false);
		
		AM.correctRotation();				//turn towards ascent zone
		AM.driveTicks(0, 0, -800);
		
		AM.sendLiftTo(1300);				//move lift up for lv 1 ascent and 
		AM.driveCM(960, false);				//drive forward to park in ascent zone
	}
	
}
