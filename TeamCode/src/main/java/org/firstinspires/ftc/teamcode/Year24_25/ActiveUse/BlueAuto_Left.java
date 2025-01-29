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

@Autonomous(name="Blue Alliance - Left Side Autonomous")
public class BlueAuto_Left extends LinearOpMode {
    AutoMethods am;

    /*
    These next few InstantFunction classes are used as a way for us to
    pass methods from AutoMethods into the Trajectory/RR Path.
    For now, they are mainly used to move the lift and servos around mid path.
     */
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

        // The entire creation of the left path on Blue Side
        Action leftPathBlue = drive.actionBuilder(beginPose)
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
                    .stopAndAdd(new SendLiftTo(InitVars.BOTTOM_PRESET))
                    .lineToY(48)

                // get in position to grab rightmost sample
                    .splineToConstantHeading(new Vector2d(48, 52), Math.toRadians(90))
                    .stopAndAdd(new LowerElbow())
                    .splineToConstantHeading(new Vector2d(48, 48), Math.toRadians(90))

                // move lift down, close claw on sample, and wait for servo to be done
                    .stopAndAdd(new SendLiftTo(InitVars.VIPER_HOME))
                    .stopAndAdd(new WaitForLift())
                    .stopAndAdd(new CloseClaw())
                    .waitSeconds(0.5)

                // setup lift+arm for basket-drop
                    .stopAndAdd(new RaiseElbow())
                    .stopAndAdd(new SendLiftTo(InitVars.TOP_PRESET))

                // turn and face basket and wait for lift to rise
                    .splineToLinearHeading(new Pose2d(48, 48, Math.toRadians(45)), Math.toRadians(-135))
                    .stopAndAdd(new WaitForLift())

                // drop sample and lower lift
                    .stopAndAdd(new OpenClaw())
                    .splineToConstantHeading(new Vector2d(46, 46), Math.toRadians(45))
                    .stopAndAdd(new SendLiftTo(InitVars.BOTTOM_PRESET))
                    .stopAndAdd(new WaitForLift())

                // get in position to grab middle sample
                    .splineToLinearHeading(new Pose2d(58, 52, Math.toRadians(-90)), Math.toRadians(-135))
                    .stopAndAdd(new LowerElbow())
                    .splineToConstantHeading(new Vector2d(58, 47), Math.toRadians(90))

                // move lift down, close claw on sample, and wait for servo to be done
                    .stopAndAdd(new SendLiftTo(InitVars.VIPER_HOME))
                    .stopAndAdd(new WaitForLift())
                    .stopAndAdd(new CloseClaw())
                    .waitSeconds(0.5)

                // setup lift+arm for basket-drop
                    .stopAndAdd(new RaiseElbow())
                    .stopAndAdd(new SendLiftTo(InitVars.TOP_PRESET))

                // turn and face basket and wait for lift to rise
                    .splineToLinearHeading(new Pose2d(49, 49, Math.toRadians(45)), Math.toRadians(-135))
                    .stopAndAdd(new WaitForLift())

                // drop sample and lower lift
                    .stopAndAdd(new OpenClaw())
                    .splineToConstantHeading(new Vector2d(46, 46), Math.toRadians(45))
                    .stopAndAdd(new SendLiftTo(InitVars.MID_PRESET))

                // drive to park in ascent zone
                    .splineToLinearHeading(new Pose2d(45, 18, 0), 0)
                    .splineToLinearHeading(new Pose2d(25, 11, Math.toRadians(180)), Math.toRadians(135))

                .build();

        Actions.runBlocking(new SequentialAction(leftPathBlue));
    }
}
