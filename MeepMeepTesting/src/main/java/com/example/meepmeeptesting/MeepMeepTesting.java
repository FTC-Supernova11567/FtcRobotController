package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(32.07237578240235, 39.4224324932042, 3.7262222290039064, Math.toRadians(197.83477704886445), 10.63)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-35, -58, Math.toRadians(90)))
                                        .setVelConstraint(new TranslationalVelocityConstraint(15))
                                        .forward(46)
                                        .waitSeconds(2)
                                        .addDisplacementMarker(() ->{
//                                            arm.top();
                                        })
                                        .waitSeconds(3)
                                        .turn(Math.toRadians(-45))
                                        .addTemporalMarker(17, () -> {
//                                            gripper.Open();
                                        })
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}