package org.firstinspires.ftc.teamcode.Year24_25.ActiveUse;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

/* This path will start on the right and side of the submersible while touching the wall. It will
    place a specimen on the top rung of the sub, then strafe left to push two samples on the ground
    into the observation zone. It will end parked at a level one ascent on te closer end of the
    submersible to the driver, coach, and human player. -CS
*/

@Autonomous(name="Blue Alliance - Simple Right Side Autonomous")
public class BlueAuto_SimpleRight extends LinearOpMode {
    private AutoMethods am;

    private class OpenClaw implements InstantFunction {
        @Override
        public void run(){
            am.openClaw();
        }
    }

    private class CloseClaw implements InstantFunction{
        @Override
        public void run(){
            am.closeClaw();
        }
    }

    private class RaiseElbow implements InstantFunction{
        @Override
        public void run(){
            am.raiseElbow();
        }
    }

    private class LowerElbow implements InstantFunction{
        @Override
        public void run(){
            am.lowerElbow();
        }
    }

    private class SendLiftTo implements InstantFunction{
        int targetPos;

        public SendLiftTo(int targetPos){
            this.targetPos = targetPos;
        }

        @Override
        public void run(){
            am.sendLiftTo(targetPos);
        }
    }

    private class WaitForLift implements InstantFunction{
        @Override
        public void run(){
            am.waitForLift();
        }
    }
    private int specimens = 3;
    private int firstSpecimenX = -8;

    @Override
    public void runOpMode() throws InterruptedException {
        // setup AutoMethods to maneuver lift, elbow, and claw
        am = new AutoMethods(hardwareMap, telemetry, this);
        am.initLiftMotors();
        am.initServos();

        // set initial position on field
        Pose2d beginPose = new Pose2d(-9, 63, Math.toRadians(-90));

        // setup RoadRunner pinpoint driving to make paths
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        // don't know if this is needed, but updates pose in MecanumDrive
        drive.updatePoseEstimate();

        // Close claw before startup to hold Specimen
        am.closeClaw();

        waitForStart();

        // The entire creation of the right path on Blue Side
        Action rightPathBlue = drive.actionBuilder(beginPose)
            // set up lift to rise to upper sub rung
                .stopAndAdd(am.new RaiseElbow())
                .stopAndAdd(am.new SendLiftTo(1470))

            // move to be in front of sub and wait for lift to raise
                .splineTo(new Vector2d(-9, 43.25), Math.toRadians(-90)) // 9, 42
                .stopAndAdd(am.new WaitForLift())

            // place specimen on rung
                .stopAndAdd(am.new SendLiftTo(770))
                .stopAndAdd(am.new WaitForLift())
                .stopAndAdd(am.new OpenClaw())

            // move lift down and back up
                .stopAndAdd(am.new SendLiftTo(InitVars.BOTTOM_PRESET))
                 .lineToY(48)

            // grab first specimen
                .splineToLinearHeading(new Pose2d(-9, 61, Math.toRadians(180)), Math.toRadians(135))
                .stopAndAdd(am.new LowerElbow())
                .lineToX(-25)
                //.stopAndAdd(am.new SendLiftTo(InitVars.VIPER_HOME))
                .stopAndAdd(am.new CloseClaw())
                .waitSeconds(0.5)

            // move back to rung
                .stopAndAdd(am.new RaiseElbow())
                .stopAndAdd(am.new SendLiftTo(InitVars.MID_PRESET))
                .splineToLinearHeading(new Pose2d(-9, 61, Math.toRadians(-90)), 45)
                .splineToConstantHeading(new Vector2d(-8, 43.25), Math.toRadians(-45))

            // hang specimen
                .stopAndAdd(am.new WaitForLift())
                .stopAndAdd(am.new SendLiftTo(770))
                .stopAndAdd(am.new WaitForLift())
                .stopAndAdd(am.new OpenClaw())

            // move lift down and back up
                .stopAndAdd(am.new SendLiftTo(InitVars.BOTTOM_PRESET))
                .lineToY(48)

            //Strafe right, move up, and strafe right again to be infront of first sample on the ground
                .lineToX(-34)
                .lineToY(19)
                .lineToX(-50)

            //Push first sample back to obsv. zone, drive back up, and strafe infront of second sample, push it to obsv. zone
                .lineToY(62)
                .lineToY(19)
                .lineToX(60)
                .lineToY(62)

            //Move up to and pivot to park at lv. 1 ascent
                .lineToY(19)
                .turn(Math.toRadians(-90))
                .stopAndAdd(am.new SendLiftTo(InitVars.MID_PRESET))
                .stopAndAdd(am.new WaitForLift())
                .lineToX(-24)

            /* grab first specimen from the ground
                .splineToLinearHeading(new Pose2d(-9, 61, Math.toRadians(180)), Math.toRadians(45))
                .stopAndAdd(am.new LowerElbow())
                .lineToX(-25)
                //.stopAndAdd(am.new SendLiftTo(InitVars.VIPER_HOME))
                .stopAndAdd(am.new CloseClaw())
                .waitSeconds(0.5)

            // move back to rung
                .stopAndAdd(am.new RaiseElbow())
                .stopAndAdd(am.new SendLiftTo(InitVars.MID_PRESET))
                .splineToLinearHeading(new Pose2d(-9, 61, Math.toRadians(-90)), 45)
                .splineToConstantHeading(new Vector2d(-7, 43.25), Math.toRadians(-45))

            // hang specimen
                .stopAndAdd(am.new WaitForLift())
                .stopAndAdd(am.new SendLiftTo(770))
                .stopAndAdd(am.new WaitForLift())
                .stopAndAdd(am.new OpenClaw())

            // move lift down and back up
                .stopAndAdd(am.new SendLiftTo(InitVars.BOTTOM_PRESET))
                .lineToY(48)

            // grab first specimen
                .splineToLinearHeading(new Pose2d(-9, 61, Math.toRadians(180)), Math.toRadians(-45))
                .stopAndAdd(am.new LowerElbow())
                .lineToX(-25)
                //.stopAndAdd(am.new SendLiftTo(InitVars.VIPER_HOME))
                .stopAndAdd(am.new CloseClaw())
                .waitSeconds(0.5)

            // move back to rung
                .stopAndAdd(am.new RaiseElbow())
                .stopAndAdd(am.new SendLiftTo(InitVars.MID_PRESET))
                .splineToLinearHeading(new Pose2d(-9, 61, Math.toRadians(-90)), 45)
                .splineToConstantHeading(new Vector2d(-6, 43.25), Math.toRadians(-45))

            // hang specimen
                .stopAndAdd(am.new WaitForLift())
                .stopAndAdd(am.new SendLiftTo(770))
                .stopAndAdd(am.new WaitForLift())
                .stopAndAdd(am.new OpenClaw())
                .stopAndAdd(am.new SendLiftTo(InitVars.BOTTOM_PRESET)) */

            //End auto path and build
                .build();
    }
}
