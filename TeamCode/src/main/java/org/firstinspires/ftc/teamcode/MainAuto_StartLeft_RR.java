package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantFunction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Main Auto - Start Left RR")
public class MainAuto_StartLeft_RR extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Must move all Java on site into Github repo to do this!!
        //AutoMethods am = new AutoMethods(hardwareMap);

        Pose2d beginPose = new Pose2d(9, 63, -Math.PI/2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        drive.updatePoseEstimate();
        telemetry.addData("Current Pose: ", drive.localizer.getPose());

        waitForStart();

        /*Action toSubmersible = drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(24, 0), 0)
                        .build();
        drive.updatePoseEstimate();*/
        Action leftPath = drive.actionBuilder(beginPose)
                /*.stopAndAdd(new InstantFunction() {
                    @Override
                    public void run() {
                        am.closeClaw()
                        am.sendLiftTo(IV.MID_PRESET);
                    }
                })*/
                .splineTo(new Vector2d(9, 39), -Math.PI/2)
                .lineToY(48)
                .splineToConstantHeading(new Vector2d(48, 48), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(48, 35), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(48, 48, Math.toRadians(45)), Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(58, 48, -Math.PI/2), Math.toRadians(-135))
                .splineToConstantHeading(new Vector2d(58, 35), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(48, 48, Math.toRadians(45)), Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(45, 18, 0), 0)
                .splineToLinearHeading(new Pose2d(25, 9, Math.PI), Math.toRadians(135))
                .build();

        Actions.runBlocking(new SequentialAction(leftPath));
    }
}
