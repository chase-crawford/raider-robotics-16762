package org.firstinspires.ftc.teamcode.Year24_25.ActiveUse;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="Auto_Left")
public class Auto_Left extends LinearOpMode {
    AutoMethods am;
    private final double elbowWait = 0.75;
    private final double specimenRungY = 39.5; // 39.5
    private final double sampleGrabY = 45.75;
    private final Pose2d basketPose = new Pose2d(50, 49.5, Math.toRadians(49));

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
                    .stopAndAdd(am.new RaiseElbow())
                    .stopAndAdd(am.new SendLiftTo(1470))

                // move to be in front of sub and wait for lift to raise
                    .lineToY(specimenRungY) // 9, 42
                    .stopAndAdd(am.new WaitForLift())

                // place specimen on rung
                    .stopAndAdd(am.new SendLiftTo(InitVars.SPECIMEN_SNAP_PRESET))
                    .stopAndAdd(am.new WaitForLift())
                    .stopAndAdd(am.new OpenClaw())

                // move lift down and back up
                    .stopAndAdd(am.new SendLiftTo(InitVars.BOTTOM_PRESET))
                    .lineToY(48)

                // get in position to grab rightmost sample
                    .splineToConstantHeading(new Vector2d(48, 52.5), Math.toRadians(90)) // y was 52
                    .stopAndAdd(am.new LowerElbow())
                    .waitSeconds(elbowWait)
                    .splineToConstantHeading(new Vector2d(48, sampleGrabY), Math.toRadians(90)) // y was 48

                // move lift down, close claw on sample, and wait for servo to be done
                    .stopAndAdd(am.new SendLiftTo(InitVars.VIPER_HOME))
                    .stopAndAdd(am.new WaitForLift())
                    .stopAndAdd(am.new CloseClaw())
                    .waitSeconds(0.5)

                // setup arm for basket-drop
                    .stopAndAdd(am.new RaiseElbow())
                    .waitSeconds(elbowWait)

                // turn and face basket and wait for lift to rise
                    .splineToLinearHeading(basketPose, Math.toRadians(-135)) //49, 49, 45deg
                    .stopAndAdd(am.new SendLiftTo(InitVars.TOP_PRESET))
                    .stopAndAdd(am.new WaitForLift())

                // drop sample and lower lift
                    .stopAndAdd(am.new OpenClaw())
                    .stopAndAdd(am.new SendLiftTo(InitVars.BOTTOM_PRESET))
                    .stopAndAdd(am.new WaitForLift())
                    //.splineToConstantHeading(new Vector2d(45, 45), Math.toRadians(47)) //46, 46

                // get in position to grab middle sample
                    .splineToLinearHeading(new Pose2d(58, 52.5, Math.toRadians(-90)), Math.toRadians(-135)) // y was 52
                    .stopAndAdd(am.new LowerElbow())
                    .waitSeconds(elbowWait)
                    .splineToConstantHeading(new Vector2d(58, sampleGrabY), Math.toRadians(90))

                // move lift down, close claw on sample, and wait for servo to be done
                    .stopAndAdd(am.new SendLiftTo(InitVars.VIPER_HOME))
                    .stopAndAdd(am.new WaitForLift())
                    .stopAndAdd(am.new CloseClaw())
                    .waitSeconds(0.5)

                // setup lift+arm for basket-drop
                    .stopAndAdd(am.new RaiseElbow())
                    .waitSeconds(elbowWait)


                // turn and face basket and wait for lift to rise
                    .splineToLinearHeading(basketPose, Math.toRadians(-135)) //49, 49
                    .stopAndAdd(am.new SendLiftTo(InitVars.TOP_PRESET))
                    .stopAndAdd(am.new WaitForLift())

                // drop sample and lower lift
                    .stopAndAdd(am.new OpenClaw())
                    .stopAndAdd(am.new SendLiftTo(InitVars.MID_PRESET))
                    .stopAndAdd(am.new WaitForLift())
                    //.splineToConstantHeading(new Vector2d(45, 45), Math.toRadians(47))

                // drive to park in ascent zone
                    .splineToLinearHeading(new Pose2d(45, 18, 0), 0)
                    .splineToLinearHeading(new Pose2d(25, 11, Math.toRadians(180)), Math.toRadians(135))
                .build();

        Actions.runBlocking(new SequentialAction(leftPathBlue));
    }
}
