package org.firstinspires.ftc.teamcode.Year24_25.ActiveUse;
import org.opencv.core.Point;

public class InitVars {
	//Position limits for viper motors -CS
		public static final int MAX_HEIGHT = 4200;
		public static final int MIN_HEIGHT = 0;
		
	//Preset positions for viper motors -CS
		public static final int TOP_PRESET = 4000;
		public static final int MID_PRESET = 1700;
		//public static final int BOTTOM_PRESET = 250; it keep hitting on the floor specimens ;-; - CC
		public static final int BOTTOM_PRESET = 300;
		public static final int HANG_PRESET = 1900;
		public static final int VIPER_HOME = 10;
		public static final int SPECIMEN_PRESET = 1360;
		public static final int SPECIMEN_SNAP_PRESET = 700;
		
	//Preset positions for elbow and claw servos -CS
		public static final double ELBOW_AUTO_START = 0.8;
		public static final double ELBOW_UP = 0.5;
		public static final double ELBOW_GRAB = 0.01;
		public static final double ELBOW_GRAB_WALL = 0.33;
		public static final double CLAW_OPEN = 0.0;
		public static final double CLAW_CLOSE = 0.3;
		
	// create variables for camera size
		public static final int CAM_W = 320;
		public static final int CAM_H = 240;
		public static final Point GRAB_POINT = new Point(CAM_W/2.0, CAM_H/2.0);
		
	//Initial speed of lift and drive motors -CS
		public static double liftSpeed = .85;
		public static double altLiftSpeed = 1.00;
		
		public static double driveSpeed = .7;			//typically .8
		public static double fineDriveSpeed = .45;		//typically .25
		
		public static final double rotSpeed = .6;
		public static final double fineRotSpeed = .35;
		public static final double autoDriveSpeed = .3;
		public static final double fineAutoDriveSpeed = 0.25;
		
	// variable for autonomous error margin with rotation/driving correction - CC
		public static final int correctRotErrorMargin = 3;
		public static final int driveErrorMargin = 2;
		
	//Vars for initial p, i, d, and f
		public static double p = 11.5,
							 i = 24, 
							 d = 3.5, 
							 f = 0;
}

