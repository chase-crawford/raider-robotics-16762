package org.firstinspires.ftc.teamcode.Year24_25.ActiveUse;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.Telemetry;

// manually input, do not delete pls - CC


@TeleOp(name="Main_TeleOp - One Driver")

public class Main_OneDriver extends OpMode{
	//Create object for InitVars and UniversalSubmethods classes
		InitVars IV = new InitVars();

	// Create object for AutoMethods class
		ManualMethods MM;

	// create sample detector vars and vision portal
		private ColorBlobLocatorProcessor blue;
		private ColorBlobLocatorProcessor red;
		private ColorBlobLocatorProcessor yellow;
		private VisionPortal portal;

	// create toggle for blue/red sample detection. Yellow is also auto allowed
		private boolean isBlueTeam;
		
	// var for storing last gamepad status
		private Gamepad prevPad = new Gamepad();

	// vars for drive style toggle
		private boolean driveGlobally;
		private boolean useSpecimens;
		
	//Final vars for autonomous driving in init (calculated in Auto_StarLeft) -CS
		final double COUNTS_PER_CM = 1.17742528;
		
	//var for current bot speed -CC
		private double currSpeed = IV.driveSpeed;
		private double currRotSpeed = IV.rotSpeed;
		private double currLiftSpeed = IV.liftSpeed;
		

	@Override
	public void init(){
		MM = new ManualMethods(hardwareMap, telemetry);
		MM.init();
		
		// FUN FACT! You can change telemetry font :)
			telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
			
		// make sure gamepad has something to show so there are
		// no nullRefErrors on startup.
			prevPad.copy(gamepad1);
			MM.sElbow0.setPosition(IV.ELBOW_UP);
	}
	

	@Override
	public void loop()
	{
	/*DRIVING: Forward, backward, strafe left, and strafe right all on gamepad1.left_stick
			   Left and right pivot on gamepad1.right_stick -CS	*/
	
		// try out speed limiting with right stick button
			limitSpeed();
		
		// Driving program from BasicDriving_Chase begins  -CS
			double vy = -gamepad1.left_stick_y * (currSpeed*0.8);				//vy = velocity left
			double vx = gamepad1.left_stick_x * currSpeed;				//vx = veloctiy right
			double w = gamepad1.right_stick_x * currRotSpeed;			//w = rotation velocity

		// Check for change of drive style - CC
			/*if (gamepad1.start && !prevPad.start)
				driveGlobally = !driveGlobally;*/

			if (gamepad1.start && !prevPad.start)
				useSpecimens = !useSpecimens;

		// Check for reset of angle - CC
			if (gamepad1.back)
				MM.resetYaw();
				
		// global direction movement (drive on the global axes of the field)
			if (driveGlobally){
				MM.driveGlobal(vx, vy, w);
			}
				
		// local direction movement (the usual) - CC
			else{
				MM.driveLocal(vx, vy, w);
			}

	/*VIPER LIFT: Send lift to preset position using buttons & 
				  let driver adjust using triggers -CS */

		/*Get input for viper lift and implement its position limits and set
		  lift to triggers for adjustment after going to preset positions -CS	*/
			float lift = (gamepad1.right_trigger - gamepad1.left_trigger);
			int newTarget = (int)(MM.getLiftTargetPosition() + lift*25);
			
			if (newTarget < IV.MAX_HEIGHT && newTarget > IV.MIN_HEIGHT){
				 MM.sendLiftTo(newTarget, currLiftSpeed);
			}

		//Get input for sElbow0 and sClaw1 -CS
			if (gamepad1.right_bumper && !prevPad.right_bumper){
				MM.toggleElbow();
			}
   
			if (gamepad1.left_bumper && !prevPad.left_bumper){
				MM.toggleClaw();
			}
			
		//Turn viper motors off if at or past MIN_POSITION -CS
			if((MM.getLiftCurrentPosition() <= IV.MIN_HEIGHT)){
				MM.setLiftPower(0);
			}

		//Send viper to preset positions using encoders -CS
			if(gamepad1.a && !prevPad.a){		
				// toggle between home position and slightly above submersible lip -CC
				if (MM.getLiftTargetPosition() == IV.BOTTOM_PRESET)
					MM.sendLiftTo(IV.VIPER_HOME, currLiftSpeed);
				
				else
					MM.sendLiftTo(IV.BOTTOM_PRESET, currLiftSpeed);
			}
			
			
			if(gamepad1.b && !prevPad.b){							//used to be VIPER_HOME

				if (useSpecimens){
					MM.sendLiftTo(IV.MID_PRESET, currLiftSpeed);
				}else {
					if (MM.getLiftTargetPosition() == IV.HANG_PRESET)
						MM.sendLiftTo(IV.VIPER_HOME, currLiftSpeed);

					else
						MM.sendLiftTo(IV.HANG_PRESET, currLiftSpeed);
				}
			}
			
			if(gamepad1.y)
				if (useSpecimens){
					MM.sendLiftTo(IV.SPECIMEN_PRESET, currLiftSpeed);
				}else {
					MM.sendLiftTo(IV.TOP_PRESET, currLiftSpeed);
				}
			
			if(gamepad1.x)
				if (useSpecimens){
					MM.sendLiftTo(770, currLiftSpeed);
				}
				else{
					MM.sendLiftTo(IV.MID_PRESET, currLiftSpeed);
				}


			if(MM.getLiftTargetPosition() != IV.VIPER_HOME || MM.getLiftCurrentPosition() > IV.VIPER_HOME)
			{
				MM.setLiftPower(IV.liftSpeed);
			}

		//Return telemtry data for motor, lift, and compass values
			MM.telemetry(vx,vy,w,driveGlobally);
			telemetry.addData("Strategy", useSpecimens ? "Specimens" : "Samples");
					  
		// set current gamepad as prev state for next iteration
			prevPad.copy(gamepad1);
	}
	
	
/***************************************** SUB-METHODS START ************************************************
							(Listed in order of importance to the best of my knowledge,
										feel free to re-arrange - CS)												   */
	/*Limits driving speed of motors using the right stick button as a toggle
		@return void
	*/
	public void limitSpeed(){
		if (gamepad1.right_stick_button && !prevPad.right_stick_button){
				if (currSpeed == IV.driveSpeed){
					currSpeed = IV.fineDriveSpeed;
					currRotSpeed = IV.fineRotSpeed;
					currLiftSpeed = IV.altLiftSpeed;
				}
				else{
					currSpeed = IV.driveSpeed;
					currRotSpeed = IV.rotSpeed;
					currLiftSpeed = IV.liftSpeed;
				}
			}
	}

}


