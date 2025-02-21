package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    final static double ELBOW_WAIT = 0.75;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            //Cady Path
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-9, 63, Math.toRadians(-90)))
                        // set up lift to rise to upper sub rung

                        // move to be in front of sub and wait for lift to raise
                            .splineTo(new Vector2d(-9, 43), Math.toRadians(-90))

                        // place specimen on rung

                        // move lift down and back up

                        //Move infront of first sample to push into obsv. zone
//                        .lineTo(new Vector2d(-28, 42))
//                        .splineToConstantHeading(new Vector2d(-45, 10), Math.PI)
//                        .lineTo(new Vector2d(-46, 10))
                        .lineTo(new Vector2d(-24, 42))
                        .splineToConstantHeading(new Vector2d(-48, 9), Math.PI)
                        //.lineTo(new Vector2d(-46, 10))

                        //Push sample into obsv. zone
                            .lineTo(new Vector2d(-46,60))

                        //Move infront of the second sample to push into obsv. zone
                            .lineTo(new Vector2d(-46, 10))
                            .lineTo(new Vector2d(-58, 10))
                            .lineTo(new Vector2d(-58, 60))
                            .waitSeconds(0.5)

                        //Spin to grab specimen off wall
                            .lineTo(new Vector2d(-58, 58))
                            .turn(Math.toRadians(180))
                            .lineTo(new Vector2d(-48,60))
                            .waitSeconds(1)

                //Chase Path (edited)

                        //Move back to rung
                            //.stopAndAdd(new SpecimenRaiseElbow())
                            .waitSeconds(ELBOW_WAIT)
                            .lineTo(new Vector2d(-48, 48))
                            //.stopAndAdd(new CloseClaw())
                            .waitSeconds(0.5)
                            //.stopAndAdd(new RaiseElbow())
                            .waitSeconds(ELBOW_WAIT)
                            .splineToLinearHeading(new Pose2d(-9, 51, Math.toRadians(-90)), Math.toRadians(45))
                            //.stopAndAdd(new SendLiftTo(InitVars.MID_PRESET))
                            .splineToConstantHeading(new Vector2d(-6, 41), Math.toRadians(-45))
                            .waitSeconds(1)

                        //Hang specimen

                        // move lift down and back up
                            //.stopAndAdd(new SendLiftTo(InitVars.VIPER_HOME))
                            .lineTo(new Vector2d(-6, 48))
                            .waitSeconds(1)

                        //Grab first specimen from wall
                            .turn(Math.toRadians(180))
                            .lineTo(new Vector2d(-48, 48))
        //                    //.stopAndAdd(new SpecimenRaiseElbow())
                            .waitSeconds(ELBOW_WAIT)
                            .lineTo(new Vector2d(-48, 51))
        //                    //.stopAndAdd(new CloseClaw())
                            .waitSeconds(0.5)

                        //Move back to rung
    //                    //.stopAndAdd(new RaiseElbow())
                        .waitSeconds(ELBOW_WAIT)
                        .splineToLinearHeading(new Pose2d(-9, 51, Math.toRadians(-90)), Math.toRadians(45))
    //                    //.stopAndAdd(new SendLiftTo(InitVars.MID_PRESET))
                        .splineToConstantHeading(new Vector2d(-3, 41), Math.toRadians(-45))
                        .waitSeconds(1)

                        //Hang specimen

                        //End at lv 1 ascent
                            .lineTo(new Vector2d(-46, 43))
                            .splineToLinearHeading(new Pose2d(-34, 5, Math.toRadians(0)), Math.PI)
                            .lineTo(new Vector2d(-25,12))

                        //End auto path and build
                            .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start();
    }
}