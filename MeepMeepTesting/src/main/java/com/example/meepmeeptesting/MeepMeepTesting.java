package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        // Declare our first bot
        //RoadRunnerBotEntity myFirstBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                //.setColorScheme(new ColorSchemeBlueDark())
                //.setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                //.followTrajectorySequence(drive ->
                        //drive.trajectorySequenceBuilder(new Pose2d(17,-73, Math.toRadians(270)))
                    //.lineToLinearHeading(new Pose2d(-33,-35,Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(-37,-12,Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(54.85,-12, Math.toRadians(0)))
//                                .lineToLinearHeading(new Pose2d(54.85,-41, Math.toRadians(0)))


                                //This is going to be from the farther side
//
//                .lineToLinearHeading(new Pose2d(-43,-30,Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(-37,-12,Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(54.85,-12, Math.toRadians(0)))
//                .lineToLinearHeading(new Pose2d(54.85,-35, Math.toRadians(0)))
//                                .build()
//                .forward(9.5)
//                .strafeRight(24)
//                .turn(Math.toRadians(180))
//                .forward(36)

//                );
        //
        RoadRunnerBotEntity redCloseRight = new DefaultBotBuilder(meepMeep)
                // We set this bot to be blue
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(16,-64, Math.toRadians(270)))
                                        .lineToLinearHeading(new Pose2d(20,-64,Math.toRadians(270)))//where the prop located (+8)
                                        .lineToLinearHeading(new Pose2d(18.5,-40,Math.toRadians(200)))//where the prop located
                                        .lineToLinearHeading(new Pose2d(17,-60, Math.toRadians(0)))
                                        .lineToLinearHeading(new Pose2d(56.85,-41.5, Math.toRadians(0)))
                                        .lineToLinearHeading(new Pose2d(54.85,-63, Math.toRadians(0)))
                                        .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                // Add both of our declared bot entities
//                .addEntity(myFirstBot)
                .addEntity(redCloseRight)
                .start();
    }
}

