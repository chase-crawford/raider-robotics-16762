package org.firstinspires.ftc.teamcode.Year24_25.ActiveUse;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class ManualMethods extends OpMethods{
	
	private boolean clawClosed, elbowDown;
	private InitVars IV = new InitVars();
	
	public ManualMethods(HardwareMap hardwareMap, Telemetry telemetry){
		super(hardwareMap, telemetry);
	}
	
	@Override
	public void initLiftMotors(){
		super.initLiftMotors();
		
		for (DcMotor motor : lift){
			motor.setTargetPosition(motor.getCurrentPosition());
		}
	}
	
	public void resetYaw(){
		compass.resetYaw();
	}
	
	/*Allows you to toggle claw button in stead of holding it down -CS
		@return void
	*/
		public void toggleClaw(){
			//sClaw1.setPosition(sClaw1.getPosition()+.02);		//Testing claw pos by incrementing it +.02 w/ each input -CS
			if (clawClosed) {
				sClaw1.setPosition(InitVars.CLAW_OPEN);
				clawClosed = false;
			} 
			
			else{
				sClaw1.setPosition(InitVars.CLAW_CLOSE);
				clawClosed = true;
			}
		}
		
	/*Allows you to toggle elbow button in stead of holding it down -CS
		@return void
	*/
		public void toggleElbow(){
			//sElbow0.setPosition(sElbow0.getPosition() + 0.02);		//Testing elbow pos by incrementing it +.02 w/ each input -CS
			if (elbowDown){
				sElbow0.setPosition(InitVars.ELBOW_UP);
				elbowDown = false;
			} 
			
			else{
				sElbow0.setPosition(InitVars.ELBOW_GRAB);
				elbowDown = true;
			}
		}

	/*Allows you to toggle elbow between three different positions
		@return void
	 */
		public void toggleElbowSpecimen() {
			int n = 0;

			if(n == 0){
				sElbow0.setPosition(IV.ELBOW_GRAB_WALL);
				n++;
			}

			else if(n == 1){
				sElbow0.setPosition(IV.ELBOW_UP);
				n++;
			}

			else{
				sElbow0.setPosition(IV.ELBOW_GRAB);
				n = 0;
			}
		}

	public void driveGlobal(double vx, double vy, double w){
		// testing with compass - CC
			double rotation = compass.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

		// setting up driving power vars - CC
			double fl,fr,rl,rr;

		// global direction movement - CC
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
			
		double[] powers = {Math.abs(fl), Math.abs(fr), Math.abs(rl), Math.abs(rr)};
			double max_power = max(powers);
			if (max_power > 1){
				fl /= max_power;
				fr /= max_power;
				rl /= max_power;
				rr /= max_power;
			}
			
		setDrivePower(rl,fl,fr,rr);
	}
	
	public void driveLocal(double vx, double vy, double w){
		
		// setting up driving power vars - CC
			double fl,fr,rl,rr;
		
		fl = vy + vx + w;					//f = forward, r = reverse
		fr = vy - vx - w;					//r = right, l = left
		rl = vy - vx + w;
		rr = vy + vx - w;
		
		double[] powers = {Math.abs(fl), Math.abs(fr), Math.abs(rl), Math.abs(rr)};
			double max_power = max(powers);
			if (max_power > 1){
				fl /= max_power;
				fr /= max_power;
				rl /= max_power;
				rr /= max_power;
			}
			
		setDrivePower(rl,fl,fr,rr);
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
	
	/* This method will output all important information to the user
	about the robots current status */
		public void telemetry(Double vx, Double vy, Double w, Boolean driveGlobally){
			
			// Lift Information
				telemetry.addData("Current Lift Position", String.format("[%,d, %,d]", lift[0].getCurrentPosition(), lift[1].getCurrentPosition()));
				telemetry.addData("Target Lift Position", String.format("[%,d, %,d]", lift[0].getTargetPosition(), lift[1].getTargetPosition()));
			
			// Servo Information
				telemetry.addLine();
				telemetry.addData("Claw Position", sClaw1.getPosition());
				telemetry.addData("Elbow Position", sElbow0.getPosition());
				
			// Drive style
				telemetry.addLine();
				if (driveGlobally != null)
					telemetry.addData("Drive Style", (driveGlobally ? "Global" : "Local"));
				else
					telemetry.addData("Drive Style", "????");
				
			// Drive vars
				telemetry.addLine();
				if (vx != null && vy != null && w != null)
					telemetry.addData("Drive Inputs (x, y, w)", String.format("(%s, %s, %s)", vx, vy, w));
				else
					telemetry.addData("Drive Inputs (x, y, w)", "????");
				
			// Rotation
				telemetry.addLine();
				telemetry.addData("Rotation/Yaw", compass.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
		}
}
	
	
	
	
	
	
	
	