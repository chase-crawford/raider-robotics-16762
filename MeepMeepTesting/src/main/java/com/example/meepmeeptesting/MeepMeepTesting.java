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
            /*Cady Path
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-9, 63, Math.toRadians(-90)))
                        // set up lift to rise to upper sub rung

                        // move to be in front of sub and wait for lift to raise
                            .splineTo(new Vector2d(-9, 43), Math.toRadians(-90))

                        // place specimen on rung

                        // move lift down and back up
                            //.lineTo(new Vector2d(-9,48))

                        //Move infront of first sample to push into obsv. zone
                            .lineTo(new Vector2d(-28, 42))
                            .splineToConstantHeading(new Vector2d(-45, 10), Math.PI)
                            .lineTo(new Vector2d(-46, 10))

                        //Push sample into obsv. zone
                            .lineTo(new Vector2d(-46,60))

                        //Move infront of the second sample to push into obsv. zone
                            .lineTo(new Vector2d(-46, 10))
                            .lineTo(new Vector2d(-58, 10))

                        //Push sample into obsv. zone
                            .lineTo(new Vector2d(-58, 60))

                        //Spin to grab specimen from wall
                            .splineToLinearHeading(new Pose2d(-46, 55, Math.toRadians(90)), Math.PI)
                            .waitSeconds(0.5)

                        //Place specimen on top rung on submersible and go back to grab second specimen
                            .splineToLinearHeading(new Pose2d(-6, 43, Math.toRadians(-90)), Math.PI/2)
                            .lineTo(new Vector2d(-6, 60))
                            .splineToLinearHeading(new Pose2d(-58, 58, Math.toRadians(90)), Math.PI)
                            .waitSeconds(0.5)

                        //Place second specimen on top rung of submersible
                            .lineTo(new Vector2d(-34, 50))
                            .splineToLinearHeading(new Pose2d(-3, 43, Math.toRadians(-90)), Math.PI/2)      //try to make angle better :/

                        //End at lv 1 ascent
                            .lineTo(new Vector2d(-46, 43))
                            .splineToLinearHeading(new Pose2d(-34, 5, Math.toRadians(0)), Math.PI)
                            .lineTo(new Vector2d(-25,12))

                        /* grab first specimen from the wall
                            .splineToLinearHeading(new Pose2d(-9, 51, Math.toRadians(90)), Math.toRadians(45))
                            .lineTo(new Vector2d(-48, 51))
                            .waitSeconds(0.5)

                        // move back to rung
                            .waitSeconds(ELBOW_WAIT)
                            .splineToLinearHeading(new Pose2d(-9, 51, Math.toRadians(-90)), 45)
                            .splineToConstantHeading(new Vector2d(-6, 43.25), Math.toRadians(-45))

                        // hang specimen

                        // move lift down and back up
                            .lineTo(new Vector2d(-9,48))

                        // grab first specimen from the wall
                            .splineToLinearHeading(new Pose2d(-9, 51, Math.toRadians(90)), Math.toRadians(45))
                            .lineTo(new Vector2d(-48, 51))
                            .waitSeconds(0.5)

                        // move back to rung
                            .splineToLinearHeading(new Pose2d(-9, 51, Math.toRadians(-90)), 45)
                            .splineToConstantHeading(new Vector2d(-3, 43.25), Math.toRadians(-45))

                        // hang specimen*/

            //Chase Path
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-9, 63, Math.toRadians(-90)))
                        // set up lift to rise to upper sub rung
                        //.stopAndAdd(new RaiseElbow())
                        //.stopAndAdd(new SendLiftTo(1470))

                        // move to be in front of sub and wait for lift to raise
                        .lineTo(new Vector2d(-9,41))
                        //.stopAndAdd(new WaitForLift())

                        // place specimen on rung
                        //.stopAndAdd(new SendLiftTo(770))
                        //.stopAndAdd(new WaitForLift())
                        //.stopAndAdd(new OpenClaw())

                        // move lift down and back up
                        //.stopAndAdd(new SendLiftTo(InitVars.VIPER_HOME))
                        .lineTo(new Vector2d(-9,48))
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
                        .splineToConstantHeading(new Vector2d(-6, 41), Math.toRadians(-45))
                        .waitSeconds(1)

                        //Hang specimen
//                    //.stopAndAdd(new WaitForLift())
//                    //.stopAndAdd(new SendLiftTo(770))
//                    //.stopAndAdd(new WaitForLift())
//                    //.stopAndAdd(new OpenClaw())

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
//                    //.stopAndAdd(new WaitForLift())
//                    //.stopAndAdd(new SendLiftTo(770))
//                    //.stopAndAdd(new WaitForLift())
//                    //.stopAndAdd(new OpenClaw())

                        //End auto path and build
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start();
    }
}