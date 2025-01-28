package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(9, 63, -Math.PI/2))
                        .splineTo(new Vector2d(9, 39), -Math.PI/2)
                        //.splineToConstantHeading(new Vector2d(9, 48), Math.toRadians(90))
                        .lineTo(new Vector2d(9, 48))
                        .splineToConstantHeading(new Vector2d(48, 48), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(48, 35), Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(48, 48, Math.toRadians(45)), Math.toRadians(-135))
                        .splineToLinearHeading(new Pose2d(58, 48, -Math.PI/2), Math.toRadians(-135))
                        .splineToConstantHeading(new Vector2d(58, 35), Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(48, 48, Math.toRadians(45)), Math.toRadians(-135))
                        .splineToLinearHeading(new Pose2d(45, 18, 0), 0)
                        .splineToLinearHeading(new Pose2d(25, 9, Math.PI), Math.toRadians(135))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}