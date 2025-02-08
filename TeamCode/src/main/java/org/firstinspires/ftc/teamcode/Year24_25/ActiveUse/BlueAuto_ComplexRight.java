package org.firstinspires.ftc.teamcode.Year24_25.ActiveUse;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="Blue Alliance - Complex Right Side Autonomous")
public class BlueAuto_ComplexRight extends LinearOpMode {
    AutoMethods am;
    final double ELBOW_WAIT = .75;

    private class OpenClaw implements InstantFunction{
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

    @Override
    public void runOpMode() throws InterruptedException {
        // setup AutoMethods to maneuver lift, elbow, and claw
            am = new AutoMethods(hardwareMap, telemetry, this);
            am.initLiftMotors();
            am.initServos();

        // set initial position on field
            Pose2d beginPose = new Pose2d(9, 63, Math.toRadians(-90));

        // setup RoadRunner pinpoint driving to make paths
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        // don't know if this is needed, but updates pose in MecanumDrive
            drive.updatePoseEstimate();

        // Close claw before startup to hold Specimen
            am.closeClaw();

        waitForStart();

        // The entire creation of the complex right path on Blue Side
            Action complexRightPathBlue = drive.actionBuilder(beginPose)
                // set up lift to rise to upper sub rung
                    .stopAndAdd(new RaiseElbow())
                    .stopAndAdd(new SendLiftTo(1470))

                // move to be in front of sub and wait for lift to raise
                    .splineTo(new Vector2d(9, 43), Math.toRadians(-90))
                    .stopAndAdd(new WaitForLift())

                // place specimen on rung
                    .stopAndAdd(new SendLiftTo(770))
                    .stopAndAdd(new WaitForLift())
                    .stopAndAdd(new OpenClaw())

                // move lift down and back up
                    .stopAndAdd(new SendLiftTo(InitVars.SPECIMEN_PRESET))
                    .lineToY(48)

                //Grab first specimen from wall
                    .splineToLinearHeading(new Pose2d(-9, 51, Math.toRadians(90)), Math.toRadians(45))
                    .stopAndAdd(new RaiseElbow())
                    .lineToX(-48)
//                    .stopAndAdd(new SendLiftTo(InitVars.VIPER_HOME))
//                    .stopAndAdd(new CloseClaw())
                    .waitSeconds(0.5)

                //Move back to rung
//                    .stopAndAdd(new RaiseElbow())
//                    .stopAndAdd(new SendLiftTo(InitVars.MID_PRESET))
                    .waitSeconds(ELBOW_WAIT)
                    .splineToLinearHeading(new Pose2d(-9, 51, Math.toRadians(-90)), 45)
                    .splineToConstantHeading(new Vector2d(-7, 43.25), Math.toRadians(-45))
                    .waitSeconds(1)

                //Hang specimen
//                    .stopAndAdd(new WaitForLift())
//                    .stopAndAdd(new SendLiftTo(770))
//                    .stopAndAdd(new WaitForLift())
//                    .stopAndAdd(new OpenClaw())

                //Move lift down and back up
//                    .stopAndAdd(new SendLiftTo(InitVars.SPECIMEN_PRESET))
                    .lineToY(48)
                    .waitSeconds(1)

                //Grab second specimen from the wall
                    .splineToLinearHeading(new Pose2d(-9, 51, Math.toRadians(90)), Math.toRadians(45))
//                    .stopAndAdd(new LowerElbow())
                    .lineToX(-48)
//                    .stopAndAdd(new SendLiftTo(InitVars.VIPER_HOME))
//                    .stopAndAdd(new CloseClaw())
//                    .waitSeconds(0.5)
                    .waitSeconds(1)

                //Move back to rung
//                    .stopAndAdd(new RaiseElbow())
//                    .stopAndAdd(new SendLiftTo(InitVars.MID_PRESET))
                    .splineToLinearHeading(new Pose2d(-9, 61, Math.toRadians(-90)), 45)
                    .splineToConstantHeading(new Vector2d(-6, 43.25), Math.toRadians(-45))
                    .waitSeconds(1)

                //Hang specimen
//                    .stopAndAdd(new WaitForLift())
//                    .stopAndAdd(new SendLiftTo(770))
//                    .stopAndAdd(new WaitForLift())
//                    .stopAndAdd(new OpenClaw())
//                    .stopAndAdd(new SendLiftTo(InitVars.BOTTOM_PRESET))

                //End auto path and build
                    .build();

        Actions.runBlocking(new SequentialAction(complexRightPathBlue));
    }
}
