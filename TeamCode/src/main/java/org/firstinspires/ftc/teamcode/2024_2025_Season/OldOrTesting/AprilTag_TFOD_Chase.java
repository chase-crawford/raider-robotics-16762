package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.text.DecimalFormat;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;


//@TeleOp(name="Autonomous-CHASE") //use Autonomous(name, group) for autonomous!

public class AprilTag_TFOD_Chase extends OpMode { //use LinearOpMode for autonomous!

		private AprilTagProcessor tagReader;
		private VisionPortal portal;
		//private TfodProcessor tfod;
		private DcMotor motor0;
		private DcMotor motor1;
		private DcMotor motor2;
		private DcMotor motor3;
		
		private AprilTagController controller;
		
		private float[] motorPowers;
		
		DecimalFormat df = new DecimalFormat("#,###.##");
		

/*		@Override												 USED FOR AUTONOMOUS
		public void runOpMode(){
				//__init__
						
				
						AprilTagProcessor tagReader = AprilTagProcessor.easyCreateWithDefaults();
						VisionPortal portal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), tagReader);
				
				//THE ACTUAL SEQUENCE OF MOVING AND S
				waitForStart();
		} */
		
		@Override
		public void init(){
				/*tfod =	new TfodProcessor.Builder()
								.build();*/
				tagReader = new AprilTagProcessor.Builder()
										.build();
										
				/*portal = new VisionPortal.Builder()
										.setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
										.enableLiveView(true)
										.addProcessor(tagReader)
										.addProcessor(tfod)
										.build();*/
				
				WebcamName cam = hardwareMap.get(WebcamName.class, "Webcam");
				controller = new AprilTagController(cam);
										
				motorPowers = new float[4];
										
				//Initialize the declared variables above by assigning them to the names given in the robot configuration on the phone
				motor0 = hardwareMap.get(DcMotor.class, "motor0");
				motor1 = hardwareMap.get(DcMotor.class, "motor1");
				motor2 = hardwareMap.get(DcMotor.class, "motor2");
				motor3 = hardwareMap.get(DcMotor.class, "motor3");
										
				//Right side motors forward, Left reverse
				motor0.setDirection(DcMotorSimple.Direction.REVERSE);
				motor1.setDirection(DcMotorSimple.Direction.REVERSE);
				motor2.setDirection(DcMotorSimple.Direction.FORWARD);
				motor3.setDirection(DcMotorSimple.Direction.FORWARD);
				
				motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
				
				motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
				
				motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
				motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
				motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
				motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		}
		
		@Override
		public void loop(){
				encoderTelemetry();
				
				// reset motor powers to 0
						for (int i=0; i<motorPowers.length; i++){
								motorPowers[i] = 0;
						}
				
				// get AprilTag (VisionPortal) & Sample (TensorFlow) detections)
						List<AprilTagDetection> detections = tagReader.getDetections();
						//List<Recognition> recognitions = tfod.getRecognitions();
				
				//processAprilTags(detections, true, true);
				
				controller.update();
				// telemetry
					DecimalFormat df = new DecimalFormat("#,###.###");
				
					telemetry.addData("Rotation", controller.rotation);
					telemetry.addData("Position on field", "("+df.format(controller.x)+", "+df.format(controller.y)+")");
					telemetry.addData("Detecting Tag?", controller.getDetectionStatus());
					telemetry.addData("Tag Reading Size", controller.tags.size());
					telemetry.addData("Tag ID", controller.tagId);
					telemetry.addData("Distance", "("+df.format(controller.distance[0])+", "+df.format(controller.distance[1])+", "+df.format(controller.distance[2]));
					
					if (controller.tags.size() >= 1){
						telemetry.addData("ftcPose", controller.tags.get(0).ftcPose);
						telemetry.addData("center", controller.tags.get(0).center);
						telemetry.addData("DataMargin", controller.tags.get(0).decisionMargin);
						telemetry.addData("metadata", controller.tags.get(0).metadata);
					}
				
				/*for(Recognition recognition : recognitions){
						//find middle coordinate of item
								double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
								double y = (recognition.getTop()	+ recognition.getBottom()) / 2 ;
						
						//adds item recognized, position on camera, and size of the item to telem
								telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
								telemetry.addData("- Position", "%.0f / %.0f", x, y);
								telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
						
						//Output ALL tfod data
								//TFODdata(recognition);
				}*/
				
				powerMotors(motorPowers);
				
				telemetry.update();
		}
		
		
		//This method takes in a recognized AprilTag
		//It will output ALL pose data relative to the webcam and tag.
				public void tagPose(AprilTagDetection tag){
						telemetry.addData("Reading Bearing", tag.ftcPose.bearing); //horizontal tilt (left/right)
						telemetry.addData("Reading Elevation", tag.ftcPose.elevation); //vertical tilt (up/down)
						telemetry.addData("Reading Range", tag.ftcPose.range); //distance in a straight line (vector)
						
						telemetry.addData("Reading Pitch", tag.ftcPose.pitch); //vertical tilt (up/down)
						telemetry.addData("Reading Roll", tag.ftcPose.roll); //barrel-roll turn
						telemetry.addData("Reading yaw", tag.ftcPose.yaw); //horizontal tilt (left/right)
						
						telemetry.addData("Reading X", tag.ftcPose.x); //distance right/left
						telemetry.addData("Reading Y", tag.ftcPose.y); //distance straight forward
						telemetry.addData("Reading Z", tag.ftcPose.z); // distance up/down
				}
				
				
		//This method takes in a recognized AprilTag
		//It will output ALL metadata of the AprilTag
				public void tagMetadata(AprilTagDetection tag){
						try{
								telemetry.addData("AprilTag ID", tag.metadata.id);
								telemetry.addData("AprilTag Size", tag.metadata.tagsize);
								telemetry.addData("AprilTag Name", tag.metadata.name);
								telemetry.addData("AprilTag Unit", tag.metadata.distanceUnit);
								telemetry.addData("AprilTag Field Position", tag.metadata.fieldPosition); //starts at RedAllianceStation
								telemetry.addData("AprilTag Field Orientation", tag.metadata.fieldOrientation); //Quaternion
						}catch(Exception e){telemetry.addData("ERROR", "Bad AprilTag");}
				}
				
				
		//This method takes in a recognized TFOD item
		//It will output ALL info of the TFOD recognition
				/*public void TFODdata(Recognition pixel){
						telemetry.addData("Width", pixel.getWidth());
						telemetry.addData("Height", pixel.getHeight());
						
				}*/

		
		//This method takes in a double-array.
		//It will move the robot the array's specified distance
				public void move_dist(float[] pows, double[] distance, double margin, float speed){
						// distance away the bot should be from where it is moving
						// currently used for aprilTags to not hit them
								double xPadding = 0;
								double yPadding = 10;
						
						// distance the bot is away from desired location
								double xDist = distance[0] - xPadding;
								double yDist = Math.abs(distance[1]) - yPadding;
								
						// scale speed depending on distance
								double ySpeedScalar = Math.pow(yDist,2)/100;
								double xSpeedScalar = Math.pow(yDist,2)/20;
								
								telemetry.addData("Distance in X",xDist);
								telemetry.addData("Distance in Y",yDist);
						
						// in order to get the desired power of the wheels, the values must
						// be normalized. To do that, we will find the angle of the vector
						// it must move in and then find the sin/cos of said angles
								double angle = Math.atan(xDist/yDist);
						
						// power for strafe and forward/backward
								double vy = Math.cos(angle)*ySpeedScalar;
								double vx = Math.sin(angle)*xSpeedScalar;
								
						// stop movement if bot is at right spot
								if (Math.abs(distance[1]) > yPadding-margin && Math.abs(distance[1]) < yPadding+margin)
										vy = 0;
										
								if (distance[0] > xPadding-margin/4 && distance[0] < xPadding+margin/4)
										vx = 0;
										
						// go backwards if too close
								if (yDist < 0)
										vy *= -1;
										
						//		if (xDist < 1)
						//				vx *= -1;

						// get power for each motor based on x AND y
								double fl = vy + vx;
								double fr = vy - vx;
								double rl = vy - vx;
								double rr = vy + vx;

						// make sure all motors are not above a power of 1/-1
								double[] powers = {Math.abs(fl), Math.abs(fr), Math.abs(rl), Math.abs(rr)};
								double max_power = max(powers);
								if (max_power > 1){
										fl /= max_power;
										fr /= max_power;
										rl /= max_power;
										rr /= max_power;
								}

						// update powers in power array
								pows[0] = (float)rl;
								pows[1] = (float)fl;
								pows[2] = (float)fr;
								pows[3] = (float)rr;
								
						telemetry.addData("Forward Speed", vy);
						telemetry.addData("Lateral Speed", vx);
				}
				
		
		//This method will set the power of the motors to the passed values
				public void powerMotors(float a, float b, float c, float d){
						motor0.setPower(a);
						motor1.setPower(b);
						motor2.setPower(c);
						motor3.setPower(d);
				}
				
				public void powerMotors(float[] pows){
						motor0.setPower(pows[0]);
						motor1.setPower(pows[1]);
						motor2.setPower(pows[2]);
						motor3.setPower(pows[3]);
				}
		
				public void encoderTelemetry(){
						telemetry.addData("Motor0 Pos", motor0.getCurrentPosition());
						telemetry.addData("Motor1 Pos", motor1.getCurrentPosition());
						telemetry.addData("Motor2 Pos", motor2.getCurrentPosition());
						telemetry.addData("Motor3 Pos", motor3.getCurrentPosition());
				}
		
				public double max(double[] powers){
				double ret = powers[0];

				for (int i=0; i<powers.length; i++){
						if (powers[i] > ret){
								ret = powers[i];
						}
				}

				return ret;
				}
				
				public void processAprilTags (List<AprilTagDetection> detections, boolean showPose, boolean showMetadata){
						for(AprilTagDetection tag : detections){
								try{ 
										//Get AprilTag position
												double[] tagPos = {tag.metadata.fieldPosition.get(0),
																								tag.metadata.fieldPosition.get(1),
																								tag.metadata.fieldPosition.get(2)};
												
										//Position of robot on field
												double[] camPos = {tagPos[0] - tag.ftcPose.x,
																								tagPos[1] - tag.ftcPose.y,
																								tagPos[2] - tag.ftcPose.z};
												
										//Add info about robot position to telemetry								
												//telemetry.addData("Camera Pos (x,y,z)", df.format(camPos[0])+", "+df.format(camPos[1])+", "+df.format(camPos[2]));
												
										//A webcam cant detect a tag when its behind it
										//So, we only need to worry about left/right
												int lateral = -1;
												if(camPos[0] > tagPos[0])
														lateral = 1;
														
														
										//Get distance bot needs to move
												double[] distance = {camPos[0] - tagPos[0],
																						camPos[1] - tagPos[1],
																						camPos[2] - tagPos[2]};
				
												//telemetry.addData("Distance (x,y,z)", df.format(distance[0])+", "+df.format(distance[1])+", "+df.format(distance[2]));
				
												//move_dist(motorPowers, distance, 2, .25f);
												
										
										//Output all of the ftcPose metadata
												if (showPose) tagPose(tag);
								
										//Output all of the aprilTag metadata
												if (showMetadata) tagMetadata(tag);
												
								}catch(Exception e){telemetry.addData("ERROR", "Bad April Tag");}
						}
				}
}
