package org.firstinspires.ftc.teamcode.Year24_25.OldOrTesting;

public class MethodStorage{
	
	/* -KC
	/// Experiental / extended method that decreases motor power the closer it
	/// is to target, similar to PIDF but just implemented in software
	private void drive4Ex(
		double motor0Target, double motor1Target,
		double motor3Target, double motor2Target
	) {
		motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			 
		int motor0Pos = (int)((motor0Target)*(COUNTS_PER_CM));
		int motor1Pos = (int)((motor1Target)*(COUNTS_PER_CM));
		int motor3Pos = (int)((motor3Target)*(COUNTS_PER_CM));
		int motor2Pos = (int)((motor2Target)*(COUNTS_PER_CM));

		motor0.setTargetPosition(motor0Pos);
		motor1.setTargetPosition(motor2Pos);
		motor3.setTargetPosition(motor0Pos);
		motor2.setTargetPosition(motor2Pos);

		motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		while(opModeIsActive()) {
			motor0.setPower((float)iv.autoDriveSpeed /
							Math.pow(motor0.getCurrentPosition() - motor0Pos, 7));
			motor1.setPower((float)iv.autoDriveSpeed /
							Math.pow(motor1.getCurrentPosition() - motor1Pos, 7));
			motor3.setPower((float)iv.autoDriveSpeed /
							Math.pow(motor3.getCurrentPosition() - motor3Pos, 7));
			motor2.setPower((float)iv.autoDriveSpeed /
							Math.pow(motor2.getCurrentPosition() - motor2Pos, 7));

			// telemetry to tell target position and current position
			telemetry.addData("Current Rotation", compass.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
			//telemetry.addData("Running Motors to (left, right)", String.format("(%,.3f, %,.3f)", leftTarget, rightTarget));
			telemetry.addData("Current Left Position", motor0.getCurrentPosition() / COUNTS_PER_CM);
			telemetry.addData("Current Right Position", motor1.getCurrentPosition() / COUNTS_PER_CM);
			telemetry.update();
			
			idle();
		}
	}*/
	
	
		
	/* CC
	// this method will take an angle and convert it into 4 powers for each motor
	// the powers are returned in the order fl,fr,br,bl and are lowered by the dampener value,
	// where 0 <= dampener <= 1.
	//			NOT CURRENTLY BEING USED IN A PROGRAM -CS
		public double[] angleToPower(double angle, double dampener){
			double[] ps = new double[4];
			
			// power for strafe and forward/backward
				double vy = Math.cos(angle)*dampener;
				double vx = Math.sin(angle)*dampener;
			
			// get power for each motor based on x AND y
				ps[0] = vy + vx;
				ps[1] = vy - vx;
				ps[2] = vy + vx;
				ps[3] = vy - vx;

			// make sure all motors are not above a power of 1/-1
				double[] powers = {Math.abs(ps[0]), Math.abs(ps[1]), Math.abs(ps[2]), Math.abs(ps[3])};
				double max_power = max(powers);
				if (max_power > 1){
					ps[0] /= max_power;
					ps[1] /= max_power;
					ps[2] /= max_power;
					ps[3] /= max_power;
				}
				
			return ps;
		}*/
		


	/***********************************************************************************************************************
		Everything beyond this point is Chase's aimbot blob stuff, idk what it does yet but I moved it down here bc of how
		big the methods are lol -CS 
	*************************************************************************************************************************/
	
	/*Initializes required stuff for blob (I think, idk I tried) -CS//
	public void initBlob(){
		 // trying to see if I can get the blob thing to work - CC
			createSampleDetectors();

		// add blob things to the vision portal
			portal = new VisionPortal.Builder()
				.addProcessor(blue)
				.addProcessor(red)
				.addProcessor(yellow)
				.setCameraResolution(new Size(iv.CAM_W, iv.CAM_H))
				.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
			.build();
	}
	
	// CC
	/* This creates our three sample detectors (for red sample, yellow sample, and blue samples) 
		to be used //
		public void createSampleDetectors(){
			blue = new ColorBlobLocatorProcessor.Builder()
				.setTargetColorRange(ColorRange.BLUE)		 // use a predefined color match
				.setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)	// exclude blobs inside blobs
				.setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // search central 1/4 of camera view
				.setDrawContours(true)						// Show contours on the Stream Preview
				.setBlurSize(5)							   // Smooth the transitions between different colors in image
			.build();

			red = new ColorBlobLocatorProcessor.Builder()
				.setTargetColorRange(ColorRange.RED)		 // use a predefined color match
				.setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)	// exclude blobs inside blobs
				.setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // search central 1/4 of camera view
				.setDrawContours(true)						// Show contours on the Stream Preview
				.setBlurSize(5)							   // Smooth the transitions between different colors in image
			.build();

			yellow = new ColorBlobLocatorProcessor.Builder()
				.setTargetColorRange(ColorRange.YELLOW)		 // use a predefined color match
				.setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)	// exclude blobs inside blobs
				.setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // search central 1/4 of camera view
				.setDrawContours(true)						// Show contours on the Stream Preview
				.setBlurSize(5)							   // Smooth the transitions between different colors in image
			.build();
		}

	// CC
	/* This will look through the camera to look for any samples in the current ROI (aka FOV)
	   It will then move the robot so that the sample is in the center of view //
		public void useAimAssist(){
			// clear telemetry to add info to it
				telemetry.clear();
				telemetry.setAutoClear(false);


			// get all preffered samples in view
				List<ColorBlobLocatorProcessor.Blob> blobs = getBlobs();
				ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);

			// make sure there are blobs to check
				if (blobs.size() == 0){
					telemetry.addLine("ERROR: Could not find any blobs in view. Terminating... ");
					telemetry.update();
					return;
				}

			// get closest blob to center
				ColorBlobLocatorProcessor.Blob closest = getClosestBlob(blobs);

				telemetry.addData("Found closest Blob! Distance to Center", getDistanceBetween(closest.getBoxFit().center, iv.GRAB_POINT));
				telemetry.update();

			// while loop to keep moving bot till blob is at center
				while (getBlobDistance(closest) > 10){
					// get closest blob's center coords
						Point blobCenter  = closest.getBoxFit().center;

					// get distance between closest blob and camera center
						Point dist = getDistanceBetween(iv.GRAB_POINT, blobCenter);

					// get angle of movement to get to cam center
						double theta = Math.atan2(dist.x, dist.y);

					// convert move angle to motor powers
						double[] movePowers = angleToPower(theta, .25);

					// power motors
						motor0.setPower(movePowers[0]);
						motor1.setPower(movePowers[1]);
						motor2.setPower(movePowers[2]);
						motor3.setPower(movePowers[3]);

					// wait a little bit in order to move motors
						//sleep(10);

					// get next detection of blobs
						blobs = getBlobs();
						ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);

						if (blobs.size() == 0){
							telemetry.addLine("ERROR: Camera lost track of all important blobs. Terminating...");
							telemetry.update();
							return;
						}

						closest = getClosestBlob(blobs);
				}

				telemetry.addLine("Reached Blob center! Closing sClaw1...");
				telemetry.update();

			// now that the camera is honed in, grab sample
				sClaw1.setPosition(InitVars.CLAW_CLOSE); //dunno the value
		}

	/* CC
	 This method will take in a collection of points and find the closest one
	 to the grabbing point //
		public ColorBlobLocatorProcessor.Blob getClosestBlob(List<ColorBlobLocatorProcessor.Blob> blobs){

			// Declaring Variables
				int closestBlobIndex = 0;
				double closestBlobDist = getBlobDistance(blobs.get(closestBlobIndex));

				int blobsSize = blobs.size();

			// For loop to find closest Blob in List to grab point
				for (int i=1; i<blobsSize; i++){

					// get ith blob & its distance from grab point
						ColorBlobLocatorProcessor.Blob nextBlob = blobs.get(i);
						RotatedRect nextBlobRect = nextBlob.getBoxFit();
						double nextBlobDist = getBlobDistance(nextBlob);
						double rectAngle = Math.abs(nextBlobRect.angle) % 180;

					// check if blob is closer than previous best and is able to be grabbed
						if (nextBlobDist < closestBlobDist && (rectAngle >= 45 || rectAngle <= 135)){
							// update closest blob values
								closestBlobIndex = i;
								closestBlobDist = getBlobDistance(blobs.get(closestBlobIndex));
						}
				}

			return blobs.get(closestBlobIndex);
		}
		
	// CC
	// this method will take a Point p and convert its x+y
	// values into it's hypotenuse value
		public double pythagPoint(Point p){
			return Math.sqrt(p.x*p.x + p.y*p.y);
		}

	// CC
	// this method will get the distance between a blob and the grabbing point
		public double getBlobDistance(ColorBlobLocatorProcessor.Blob blob){
			// get center coordinates of blob
				Point blobCenter = blob.getBoxFit().center;

			// get distance between blob and center
				Point blobCenterDist = getDistanceBetween(iv.GRAB_POINT, blobCenter);

			// return distance as the hypotenuse or direct distance
				return pythagPoint(blobCenterDist);
		}
		
	 // CC
	//this method will get the numerical distance between two points. It works by subtracting Point b from Point a.
		public Point getDistanceBetween(Point a, Point b){
			return new Point(a.x-b.x, a.y-b.y);
		}


	// CC
	// this method will get a List of all blobs in view
	// that are either colored to our alliance (red/blue) or neutral colored (yellow)
		public List<ColorBlobLocatorProcessor.Blob> getBlobs(){
			// get yellow blobs
				List<ColorBlobLocatorProcessor.Blob> ret = yellow.getBlobs();

			// add alliance blobs to List
				if (isBlueTeam)
					ret.addAll(blue.getBlobs());
				else
					ret.addAll(red.getBlobs());

			return ret;
		}*/
		
		
	/***********************************************************************************************************************
		Everything beyond this point is commented out chunks of code from the main method as of 11/22/24. 
		Moved them here incase we still need them but they stay out of the way -CS
	*************************************************************************************************************************/
		
		// Change Speed dynamically for lift -CC
			/*if (gamepad1.dpad_up && !previousDPadUp)
				liftSpeed += .1;
			if (gamepad1.dpad_down && !previousDPadDown)
				liftSpeed -= .1;

			liftSpeed = Math.max(0, Math.min(1, liftSpeed));

			previousDPadUp = gamepad1.dpad_up;
			previousDPadDown = gamepad1.dpad_down;*/
			
			
	
				// previous code
				//viper0.setTargetPosition(iv.BOTTOM_PRESET);
				//viper1.setTargetPosition(iv.BOTTOM_PRESET);
				
				
				
		/*if (gamepad1.dpad_left)
				sElbow0.setPosition(iv.ELBOW_GRAB);
			if (gamepad1.dpad_down)
				sElbow0.setPosition(iv.ELBOW_UP*1/3);
			if (gamepad1.dpad_right)
				sElbow0.setPosition(iv.ELBOW_UP*3/4);
			if (gamepad1.dpad_up)
				sElbow0.setPosition(iv.ELBOW_UP);*/
				
				
				
		/*long start = System.currentTimeMillis();
		long end = start + 1000;
		while (System.currentTimeMillis() < end)
		{
			  clawServo.setPower(.25)
		}
		clawServo.setPosition(0 - 1); */	

}
