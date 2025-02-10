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

    private class SpecimenRaiseElbow implements InstantFunction{
        @Override
        public void run(){am.raiseElbowToWall();}
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
                    .lineToY(41)
                    .stopAndAdd(new WaitForLift())

                // place specimen on rung
                    .stopAndAdd(new SendLiftTo(770))
                    .stopAndAdd(new WaitForLift())
                    .stopAndAdd(new OpenClaw())

                // move lift down and back up
                    .stopAndAdd(new SendLiftTo(InitVars.VIPER_HOME))
                    .lineToY(48)
                    .waitSeconds(1)

                //Move infront of first sample to push into obsv. zone
                    .lineToY(-42)
                    .lineToX(-28)
                    .splineToConstantHeading(new Vector2d(-45, 10), Math.PI)
                    .lineToX(-46)
                    .lineToY(10)
                    .waitSeconds(0.5)

                /*Push sample into obsv. zone
                    .lineToX(-46)
                    .lineToY(60)

                //Move infront of the second sample to push into obsv. zone
                    .lineToY(10)
                    .lineToX(-58)
                    .lineToY(60)
                    .waitSeconds(0.5)

                //Spin to grab specimen off wall
                    .lineToY(58)
                    .turn(Math.toRadians(180))
                    .lineToX(-48)
                    .lineToY(60)
                    .waitSeconds(1)

                //Move back to rung
//                  .stopAndAdd(new SpecimenRaiseElbow())
                    .waitSeconds(ELBOW_WAIT)
                    .lineToY(48)
//                  .stopAndAdd(new CloseClaw())
                    .waitSeconds(0.5)
//                  .stopAndAdd(new RaiseElbow())
                    .waitSeconds(ELBOW_WAIT)
                    .lineToX(-9)
                    .lineToY(51)
//                   .stopAndAdd(new SendLiftTo(InitVars.MID_PRESET))
                    .lineToX(-6)
                    .lineToY(41)
                    .waitSeconds(1)

                //Hang specimen
//                  .stopAndAdd(new WaitForLift())
//                  .stopAndAdd(new SendLiftTo(770))
//                  .stopAndAdd(new WaitForLift())
//                  .stopAndAdd(new OpenClaw())

                // move lift down and back up
//                  .stopAndAdd(new SendLiftTo(InitVars.VIPER_HOME))
                    .lineToY(48)
                    .waitSeconds(1)

                //Grab first specimen from wall
                    .turn(Math.toRadians(180))
                    .lineToX(-48)
//                  .stopAndAdd(new SpecimenRaiseElbow())
                    .waitSeconds(ELBOW_WAIT)
                    .lineToY(51)
//                  .stopAndAdd(new CloseClaw())
                    .waitSeconds(0.5)

                //Move back to rung
//                  .stopAndAdd(new RaiseElbow())
                    .waitSeconds(ELBOW_WAIT)
                    .lineToX(-9)
//                  .stopAndAdd(new SendLiftTo(InitVars.MID_PRESET))
                    .lineToX(-3)
                    .lineToY(41)
                    .waitSeconds(1)

                //Hang specimen
//                    .stopAndAdd(new WaitForLift())
//                    .stopAndAdd(new SendLiftTo(770))
//                    .stopAndAdd(new WaitForLift())
//                    .stopAndAdd(new OpenClaw())

                //End at lv 1 ascent
                    .lineToX(-46)
                    .lineToY(43)
                    .splineToLinearHeading(new Pose2d(-34, 5, Math.toRadians(0)), Math.PI)
                    .lineToX(-25)
                    .lineToY(12)

                //End auto path and build*/
                    .build();

        Actions.runBlocking(new SequentialAction(complexRightPathBlue));
    }
}
