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
            /*CHASE PATH
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
            .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(9, 63, Math.PI/2))
                    // set up lift to rise to upper sub rung
                    //(new RaiseElbow())
                    //(new SendLiftTo(1470))

                    // move to be in front of sub and wait for lift to raise
                    .splineTo(new Vector2d(9, 42), Math.toRadians(-90))
                    //(new WaitForLift())

                    // place specimen on rung
                    //(new SendLiftTo(770))
                    //(new WaitForLift())
                    //(new OpenClaw())

                    // move lift down and back up
                    //(new SendLiftTo(InitVars.BOTTOM_PRESET))
                    .lineTo(new Vector2d(9,48))

                    // get in position to grab rightmost sample
                    .splineToConstantHeading(new Vector2d(48, 52), Math.toRadians(90))
                    //(new LowerElbow())
                    .splineToConstantHeading(new Vector2d(48, 48), Math.toRadians(90))

                    // move lift down, close claw on sample, and wait for servo to be done
                    //(new SendLiftTo(InitVars.VIPER_HOME))
                    //(new WaitForLift())
                    //(new CloseClaw())
                    .waitSeconds(0.5)

                    // setup lift+arm for basket-drop
                    //(new RaiseElbow())
                    //(new SendLiftTo(InitVars.TOP_PRESET))

                    // turn and face basket and wait for lift to rise
                    .splineToLinearHeading(new Pose2d(48, 48, Math.toRadians(45)), Math.toRadians(-135))
                    //(new WaitForLift())

                    // drop sample and lower lift
                    //(new OpenClaw())
                    .splineToConstantHeading(new Vector2d(46, 46), Math.toRadians(45))
                    //(new SendLiftTo(InitVars.BOTTOM_PRESET))
                    //(new WaitForLift())

                    // get in position to grab middle sample
                    .splineToLinearHeading(new Pose2d(58, 52, Math.toRadians(-90)), Math.toRadians(-135))
                    //(new LowerElbow())
                    .splineToConstantHeading(new Vector2d(58, 48), Math.toRadians(90))

                    // move lift down, close claw on sample, and wait for servo to be done
                    //(new SendLiftTo(InitVars.VIPER_HOME))
                    //(new WaitForLift())
                    //(new CloseClaw())
                    .waitSeconds(0.5)

                    // setup lift+arm for basket-drop
                    //(new RaiseElbow())
                    //(new SendLiftTo(InitVars.TOP_PRESET))

                    // turn and face basket and wait for lift to rise
                    .splineToLinearHeading(new Pose2d(48, 48, Math.toRadians(45)), Math.toRadians(-135))
                    //(new WaitForLift())

                    // drop sample and lower lift
                    //(new OpenClaw())
                    .splineToConstantHeading(new Vector2d(46, 46), Math.toRadians(45))
                    //(new SendLiftTo(InitVars.MID_PRESET))

                    // drive to park in ascent zone
                    .splineToLinearHeading(new Pose2d(45, 18, 0), 0)
                    .splineToLinearHeading(new Pose2d(25, 11, Math.toRadians(180)), Math.toRadians(135))*/

            //CADY PATH
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        .forward(30)
                        .turn(Math.toRadians(90))
                        .forward(30)
                        .turn(Math.toRadians(90))
                        .forward(30)
                        .turn(Math.toRadians(90))
                        .forward(30)
                        .turn(Math.toRadians(90))


            .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}