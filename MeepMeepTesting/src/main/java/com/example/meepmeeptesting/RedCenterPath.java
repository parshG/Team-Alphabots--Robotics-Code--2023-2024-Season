package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import jdk.tools.jlink.internal.plugins.VendorVersionPlugin;

public class RedCenterPath {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 175, Math.PI, Math.PI, 13.9)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -62, Math.toRadians(90)))
                                .setReversed(true)
                                .strafeTo(new Vector2d(-36, -45))
                                .splineToSplineHeading(new Pose2d(-36,-37, Math.toRadians(0)), Math.toRadians(-90)) //x was -38 before
                                .setReversed(true)
                                .setTangent(0)
                                .splineToConstantHeading(new Vector2d(-38, -14), Math.toRadians(0))
                                .splineTo(new Vector2d(20, -14), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(54,-37.5, Math.toRadians(180)), Math.toRadians(0))
                                .strafeTo(new Vector2d(60.0, -37.5))

                                .build()

                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
