package org.firstinspires.ftc.teamcode.Year24_25.ActiveUse;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//@Autonomous(name = "Autonomous_StartLeft")
public class Autonomous_StartLeft extends LinearOpMode{
	//Create object for InitVars class -CS
		InitVars IV = new InitVars();

	// Create object for AutoMethods class
		AutoMethods AM;
					  
					  
	//Initilize all motor and servo values -CS								  
		@Override
		public void runOpMode()
		{
			telemetry.setAutoClear(false);
			AM = new AutoMethods(hardwareMap, telemetry, this);
			AM.init();
			
			//Begin driving program
			/*	STEPS FOR AUTONOMOUS PATH
					1. keep elbow in upright position, close claw on specimen -CC
			
					WAIT FOR START
					
					2. drive forward to be in front of submersible
					
					4. wait for lift to get to high bar position 
					5. move lift down to snap specimen to bar 
					
					6. wait for lift to move down 
					7. let specimen go 
					
					8. bring lift down before continuing to drive 
					10. move robot to left of submersible
					11. move forward to be behind + to the right of alliance samples - CC
					
					12. drive to be behind rightmost sample
					13. push right sample into net zone
					14. drive back behind samples
					15. push middle sample to net zone
					
					16. drive backwards to middle of field
					17. drive left to get in ascent zone
			*/
			
		//AM.raiseElbow();
		//AM.openClaw();
		AM.startWithSpecimen();
		
		waitForStart();
		
		// hang specimen on high rung
			AM.raiseElbow();
			hangHeldSpecimen();
		
		// move to and push rightmost neutral sample into net zone
			//pushRightSample();
			dunkRightSample();
		
		//TEMP PATH - Park in net after moving first sample -CS
			//driveHorizontal(-650);
		
		// drive back forward, strafe left, and push middle sample into net zone
		// then, drive forward and park in ascent zone
			//pushMiddleSampleAndPark();
			dunkMiddleSample();

	}
	
	// This method will move the robot from the starting position to the
	// specimen rung hanging area and attach a blue specimen to the highest rung.
	// PRECONDITION: Robot is holding a specimen
		public void hangHeldSpecimen(){
			/*AM.sendLiftTo(1470);						//iv.MID_PRESET = 1700
			AM.driveCM(580);
			AM.waitForLift();
			//AM.driveCM(TILE/2, 645);					//started at 775
			
			AM.sendLiftTo(1470-680);
			AM.waitForLift();
			AM.openClaw();	
			
			AM.sendLiftTo(IV.VIPER_HOME);			
			AM.waitForLift();*/
			
			// drive forward to rung and set up lift to place
				AM.sendLiftTo(1470);
				AM.driveTicks(760);
				AM.waitForLift();
			
			// snap specimen to rung and then reset lift
				AM.sendLiftTo(1470-700);
				AM.waitForLift();
				AM.openClaw();
				AM.sendLiftTo(IV.VIPER_HOME);
		}
			
			
	// This method will move from the specimen rungs and drive behind the
	// leftmost neutral sample set. It will then push the rightmost sample into the net zone.
	// PRECONDITION: Bot has moved to the specimen rung
		public void pushRightSample(){
			/*AM.driveCM(-900, -25);
			//AM.correctRotation();
			
			AM.driveTicks(4800);			//moving back too fast when pushing sample
			//AM.driveTicks(0,0,800);
			/*AM.driveCM(-270, 0);
			AM.correctRotation();
			AM.driveCM(-1650);*/
			
			// Drive horizontal from submersible and drive behind right sample
				AM.driveTicks(-900, 0);
				AM.driveTicks(900);
				AM.driveTicks(-450, 0);
				
			// make sure robot is facing forward
				AM.correctRotation();
				
			// Push right sample back
				AM.driveTicks(-1450);
		}
		
	// This method will move from the specimen rungs and drive in front of the
	// leftmost neutral sample set. It will then grab the rightmost sample and place it in the
	// high net.
	// PRECONDITION: Bot has moved to the specimen rung
	// POSTCONDITION: Bot is at same end spot as pushRightSpecimen()
		public void dunkRightSample(){
			/* Raise lift to not bounce off sample/floor
			strafe to be facing right sample
			lower elbow and lift to grab sample */
				AM.sendLiftTo(IV.BOTTOM_PRESET);
				AM.driveTicks(-1500, 30);
				AM.lowerElbow();
				AM.drive(-185);
				AM.sendLiftTo(IV.VIPER_HOME);
				AM.waitForLift();
				AM.correctRotation();
			
			/* grab sample, back up to give close time
			raise elbow and turn around to face bucket */
				AM.closeClaw();
				AM.driveTicks(-60);
				AM.driveTicks(-40); // was 60, then Simon tried to lift WHOLE basket in match 12 - CC
				AM.raiseElbow();
				AM.driveTicks(0,0,-1200);
			
			/* Make sure elbow is up, start raising lift
			drive to be in line with basket
			wait, the let the sample drop */
				AM.raiseElbow();
				AM.sendLiftTo(IV.TOP_PRESET);
				AM.driveTicks(-90, 135);
				AM.waitForLift();
				AM.openClaw();
				
			// TEMP - Back up and slow lower lift
				/*AM.driveTicks(-500);
				AM.sendLiftTo(IV.VIPER_HOME);
				AM.waitForLift();*/
				
		}
		
	// this method will drive in front of the middle sample and
	// dunk it into the top basket
	// PRECONDITION: dunkRightSample() has already been called
		public void dunkMiddleSample(){
			/* Drive around middle sample to be behind it
			Lower lift and close claw
			Pick up sample */
				AM.sendLiftTo(IV.BOTTOM_PRESET);
				AM.driveTicks(0, 0, 1200);
				AM.lowerElbow();
				AM.driveTicks(-460, 130);
				
				AM.waitForLift();
				AM.sendLiftTo(IV.VIPER_HOME);
				AM.correctRotation();
				
				AM.waitForLift();
				AM.closeClaw();
				AM.driveTicks(-50);
				AM.driveTicks(480, -75);
				
				AM.raiseElbow();
				AM.driveTicks(0, 80, -1200);
				AM.sendLiftTo(IV.TOP_PRESET);
				//AM.driveTicks(-15);
				AM.waitForLift();
				AM.openClaw();
				
			// TEMP - Back up and slow lower lift
				AM.driveTicks(-500);
				AM.sendLiftTo(1300);
				AM.driveTicks(0, 0, -1200);
				AM.driveTicks(-1100, 0);
				AM.driveTicks(0, 650);
				AM.waitForLift();
		}


	// this method will drive behind the floor samples, push the middle one into
	// the net zone, and then park for a level one ascent.
	// PRECONDITION: Bot has pushed the rightmost sample into the net zone already
		public void pushMiddleSampleAndPark(){
			/*AM.correctRotation();
			AM.driveCM(1650);	
			AM.correctRotation();
			AM.driveCM(-270, 0);
			
			AM.correctRotation();
			AM.driveCM(-1650);
			AM.correctRotation();
	
			AM.driveCM(1650);
			AM.driveCM(0, 0, 700);
			AM.correctRotation(-90);
			AM.sendLiftTo(IV.MID_PRESET-400);
			AM.driveCM(640);*/
			
			// Drive forward and then left to be behind mid sample
				AM.driveTicks(1450);
				AM.driveTicks(-450, 0);
				
			// Push sample back and then go to previous position
				AM.driveTicks(-1450);
				AM.driveTicks(1450);
				
			// Turn to face submersible, drive in ascent zone
				AM.driveTicks(0, 0, 800);
				AM.driveTicks(600);
		}
}


