package org.firstinspires.ftc.teamcode.Year24_25.OldOrTesting;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.List;


public class AprilTagController {

	private AprilTagProcessor reader;
	private VisionPortal portal;
	
	public List<AprilTagDetection> tags;
	public boolean seesTag;
	public static int MAX_WIDTH = 144; // x
	public static int MAX_HEIGHT = 144; // y
	// -1 to 1
	public double x;
	// -1 to 1
	public double y;
	// In degrees from [0, 360]
	public double rotation;
	public int tagId;
	public double[] distance = new double[3];

	public AprilTagController(WebcamName cam){
		reader = new AprilTagProcessor.Builder()
					//.setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
					.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
					.build();
					
		portal = new VisionPortal.Builder()
					.setCamera(cam)
					.enableLiveView(true)
					.addProcessor(reader)
					.build();
	}

	public void update(int tagId){
		tags = reader.getDetections();

		seesTag = false;
		for (AprilTagDetection tag : tags){
			if (tag.metadata.id != tagId){
				continue;
			}

			try{
				tagId = tag.metadata.id;
				
			//Get AprilTag position
				double[] tagPos = {tag.metadata.fieldPosition.get(0),
									tag.metadata.fieldPosition.get(1),
									tag.metadata.fieldPosition.get(2)};

			//Position of robot on field
				double[] camPos = {tagPos[0] - tag.ftcPose.x,
									tagPos[1] - tag.ftcPose.y,
									tagPos[2] - tag.ftcPose.z};

				// KJC field is 11.75 ft x 11.75 ft
				// (TODO: just measure one pad way easier and better)
			// update object fields
				x = camPos[0];
				y = camPos[1];

			// fun quaternion work... - CC
				Quaternion q = tag.metadata.fieldOrientation;
				//rotation = Math.toDegrees(2*Math.acos(q.w)) + tag.ftcPose.yaw;
				double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
				double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
				rotation = Math.toDegrees(Math.atan2(siny_cosp, cosy_cosp)) + tag.ftcPose.yaw;
			
				seesTag = true;
			} catch(Exception e){}
			
			return;
		}
	}

	public void update(){
		tags = reader.getDetections();

		if (tags.size() == 0){
			seesTag = false;
			return;
		}
		seesTag = true;
		

		AprilTagDetection tag = tags.get(0);

		try{
			tagId = tag.id;
			
		//Distance away from tag
			distance[0] = tag.rawPose.x;
			distance[1] = tag.rawPose.y;
			distance[2] = tag.rawPose.z;
			
		//Get AprilTag position
			double[] tagPos = {tag.metadata.fieldPosition.get(0),
								tag.metadata.fieldPosition.get(1),
								tag.metadata.fieldPosition.get(2)};

		//Position of robot on field
			double[] camPos = {tagPos[0] - tag.ftcPose.x,
								tagPos[1] - tag.ftcPose.y,
								tagPos[2] - tag.ftcPose.z};

		// update object fields
			x = camPos[0]/MAX_WIDTH;
			y = camPos[1]/MAX_HEIGHT;

		// fun quaternion work... - CC
			Quaternion q = tag.metadata.fieldOrientation;
			//rotation = Math.toDegrees(2*Math.acos(q.w)) + tag.ftcPose.yaw;
			double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
			double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
			rotation = Math.toDegrees(Math.atan2(siny_cosp, cosy_cosp))-90 + tag.ftcPose.yaw;
			if (rotation < 0)
				rotation += 360;
		} catch (Exception e){}



	}

	// returns a list of AprilTags that the robot currently or has last seen.
		public List<AprilTagDetection> getTagDetections(){
			return tags;
		}

	// returns a boolean that corresponds to if the user currently sees any tags
		public boolean getDetectionStatus(){
			return seesTag;
		}

	/*
		instatiatable class

		- methods to return num -1 to 1 about relative position (x and y doubles)
			of robot. 0 is the center. Make 1,1 red observation zone

		- getEstimatedPosition() - return the approximate position of robot

		- variable to see if robot is reading an aprilTag

		- update: process all of the AprilTag reading

		- get rotational information: 0,0 is facing the whiteboard

	*/
}
