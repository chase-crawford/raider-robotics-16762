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

@Autonomous(name="Auto_Right")
public class Auto_Right extends LinearOpMode {
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
            Pose2d beginPose = new Pose2d(-9, 63, Math.toRadians(-90));

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
                    .lineToY(40)
                    .stopAndAdd(new WaitForLift())

                // place specimen on rung
                    .stopAndAdd(new SendLiftTo(700))
                    .stopAndAdd(new WaitForLift())
                    .waitSeconds(0.5)
                    .stopAndAdd(new OpenClaw())

                // move lift down and back up
                    .stopAndAdd(new SendLiftTo(InitVars.VIPER_HOME))

                //Move infront of first sample to push into obsv. zone
                    .strafeTo(new Vector2d(-26, 48))
                    .splineToConstantHeading(new Vector2d(-46, 9), Math.PI)

                //Push sample into obsv. zone
                    .strafeTo(new Vector2d(-46, 58))

                //Cutting this to save time
                /*Move infront of the second sample to push into obsv. zone
                    .lineToY(10)
                    .strafeTo(new Vector2d(-55, 10))
                    .strafeTo(new Vector2d(-55, 58))
                    .waitSeconds(0.5)*/

                //Raise lift and lower arm
                    .stopAndAdd(new SendLiftTo(1555))
                    .stopAndAdd(new LowerElbow())

                //Spin to grab specimen
                    .strafeTo(new Vector2d(-46, 51))
                    .turn(Math.toRadians(180))

                //Grab specimen off wall
                    //.waitSeconds(1)
                    .stopAndAdd(new CloseClaw())
                    .waitSeconds(ELBOW_WAIT)
                    .stopAndAdd(new RaiseElbow())

                //Move back to submersible and raise lift
                    .stopAndAdd(new SendLiftTo(InitVars.MID_PRESET))
                    .splineToLinearHeading(new Pose2d(-9, 40, Math.toRadians(-90)), Math.toRadians(45)) //
                    .strafeTo(new Vector2d(-6, 40))

                //Hang specimen
                    .stopAndAdd(new SendLiftTo(700))
                    //.waitSeconds(ELBOW_WAIT)
                    .stopAndAdd(new WaitForLift())
                    .stopAndAdd(new OpenClaw())

                // move lift down and back up
                    .stopAndAdd(new SendLiftTo(InitVars.VIPER_HOME))
                    //.waitSeconds(0.5)
                    .lineToY(48)

                //Get back to wall while raising lift for second specimen
                    .stopAndAdd(new SendLiftTo(1555))
                    .stopAndAdd(new LowerElbow())
                    .splineToLinearHeading(new Pose2d(-46, 51, Math.toRadians(90)), Math.toRadians(45)) //

                //Grab specimen from wall
                    .stopAndAdd(new CloseClaw())
                    .waitSeconds(ELBOW_WAIT)
                    .stopAndAdd(new RaiseElbow())

                //Move back to submersible and raise lift
                    .stopAndAdd(new SendLiftTo(InitVars.MID_PRESET))
                    .splineToLinearHeading(new Pose2d(-9, 40, Math.toRadians(-90)), Math.toRadians(45)) //
                    .strafeTo(new Vector2d(-6, 40))

                //Hang specimen
                    .stopAndAdd(new SendLiftTo(700))
                    //.waitSeconds(ELBOW_WAIT)
                    .stopAndAdd(new WaitForLift())
                    .stopAndAdd(new OpenClaw())

                // move lift down and back up
                    .stopAndAdd(new SendLiftTo(InitVars.VIPER_HOME))
                    //.waitSeconds(0.5)
                    //.lineToY(48)

                //End parked infront of submersible rungs
                    .lineToY(48)

                /*End parked at lv 1 ascent
                    .strafeTo(new Vector2d(-34, 48))
                    .turn(Math.toRadians(-90))
                    .strafeTo(new Vector2d(-25, 43))
                    .strafeTo(new Vector2d(-25, 12))

                //End auto path and build   */
                    .build();

        //http://192.168.43.1:8080/dash

        Actions.runBlocking(new SequentialAction(complexRightPathBlue));
    }
}
