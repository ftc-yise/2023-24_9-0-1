package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity blueExterior = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setDimensions(14.125, 16)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10.38)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-39.4375, 62, Math.toRadians(90)))
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(-41, 37, Math.toRadians(90)))
                                .strafeLeft(10)
                                .splineTo(new Vector2d(-50, 12), Math.toRadians(-90))
                                .back(50)
                                .splineTo(new Vector2d(48, 36), Math.toRadians(0))
                                .forward(1)
                                .splineTo(new Vector2d(-5, 12), Math.toRadians(180))
                                .forward(50)
                                .build()
                );

        RoadRunnerBotEntity testBot = new DefaultBotBuilder(meepMeep)
                // 3We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setDimensions(14.125, 16)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10.38)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-43.5, 38, Math.toRadians(90)))
                                .back(-16)
                                .lineToLinearHeading(new Pose2d(-55, 36, Math.toRadians(180)))
                                .forward(5)
                                .waitSeconds(2) // insert intake here
                                .back(3)
                                .strafeLeft(24)
                                .lineToLinearHeading(new Pose2d(18, 12, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(46, 42, Math.toRadians(0)))                                .build()
                );

        RoadRunnerBotEntity blueExteriorLeft = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setDimensions(14.125, 16)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10.38)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-39.4375, 62, Math.toRadians(90)))
                                .waitSeconds(6)
                                .setReversed(true)
                                .lineToLinearHeading(new Pose2d(-30, 43, Math.toRadians(120)))
                                .strafeLeft(10)
                                .splineTo(new Vector2d(-50, 12), Math.toRadians(-90))
                                .forward(5)
                                .back(50)
                                .splineTo(new Vector2d(48, 36), Math.toRadians(0))
                                .build()
                );

        RoadRunnerBotEntity blueInterior = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setDimensions(14.125, 16)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10.38)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(15.4375, 62, Math.toRadians(90)))
                                .waitSeconds(3.71)
                                .setReversed(true)
                                .splineTo(new Vector2d(17, 37), Math.toRadians(0))
                                .lineToLinearHeading(new Pose2d(48, 38, Math.toRadians(0)))
                                .forward(1)
                                .splineTo(new Vector2d(-5, 12), Math.toRadians(180))
                                .forward(50)
                                .build()
                );

        RoadRunnerBotEntity redExterior = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10.38)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-41.5, -61, Math.toRadians(-90)))
                                .splineTo(new Vector2d(-30.0, 30.0), Math.toRadians(180))
                                .splineTo(new Vector2d(-60.0, 0.0), Math.toRadians(-90))
                                .splineTo(new Vector2d(-30.0, -30.0), Math.toRadians(0))
                                .splineTo(new Vector2d(0.0, 0.0), Math.toRadians(90))
                                /*.setReversed(true)
                                .lineToLinearHeading(new Pose2d(-41, -37, Math.toRadians(-90)))
                                .strafeRight(10)
                                .splineTo(new Vector2d(-50, -12), Math.toRadians(90))
                                .back(45)
                                .splineTo(new Vector2d(48, -38), Math.toRadians(0))
                                .forward(1)
                                .splineTo(new Vector2d(-5, -12), Math.toRadians(180))
                                .forward(50)*/
                                .build()
                );

        RoadRunnerBotEntity redInterior = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeRedDark())
                .setDimensions(14.125, 16)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10.38)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-39.4375, -62, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(-55, -12, Math.toRadians(180)))
                                .back(60)
                                .splineTo(new Vector2d(47, -31), Math.toRadians(0))
                                .setReversed(false)
                                .splineTo(new Vector2d(5, -12), Math.toRadians(180))
                                .forward(60)
                                .back(60)
                                .splineTo(new Vector2d(47, -31), Math.toRadians(0))
                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                // Add both of our declared bot entities
                //.addEntity(blueInterior)
                //.addEntity(blueExterior)
                //.addEntity(redExterior)
                .addEntity(redInterior)
                //.addEntity(testBot)
                .start();
    }
}