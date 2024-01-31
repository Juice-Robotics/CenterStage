package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);
        RoadRunnerBotEntity myBotCycleSafeBlue = new DefaultBotBuilder(meepMeep)
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14, 17)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-62, -34, Math.toRadians(180)))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-34, -32, Math.toRadians(235)), Math.toRadians(30))
                        .forward(12)
                        .turn(Math.toRadians(35))
                        .strafeRight(6)
                        //.splineToLinearHeading(new Pose2d(-48, -40, Math.toRadians(235)), Math.toRadians(30))
                        //.splineToLinearHeading(new Pose2d(-57, -40, Math.toRadians(-90)), Math.toRadians(-90))
                        //.setReversed(false)
                        //.splineToConstantHeading(new Vector2d(-57, -25), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-57, 10), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-39, 50), Math.toRadians(90))
                        .build()
                );


        RoadRunnerBotEntity myBotCyclesSafeRed = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(55, 55, Math.toRadians(360), Math.toRadians(360), 13.28)
                .setDimensions(15, 17)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-62, 13, Math.toRadians(180)))
                                        .setReversed(true)
                                        .splineToLinearHeading(new Pose2d(-37.5, 6, Math.toRadians(145)), Math.toRadians(-32))
                                        .setReversed(false)
                                        .splineToLinearHeading(new Pose2d(-30, 51, Math.toRadians(270)), Math.toRadians(90))
                                        .addTemporalMarker(0, () -> {
                                        })
                                        .addTemporalMarker(1.1, () -> {
                                        })
                                        .addTemporalMarker(2.35, () -> {
                                        })
                                        .addTemporalMarker(3, () -> {
                                        })
                                        .waitSeconds(1)
                                        .strafeRight(28)
                                        .back(5)
                                        .waitSeconds(3)
                                        .build()

                );


        Image img = null;
        try { img = ImageIO.read(new File("/Users/huntert/Downloads/Juice-CENTERSTAGE-Dark.png")); }
//        try { img = ImageIO.read(new File("/Users/siddharth/dev/Juice/CenterStage/MeepMeepTesting/src/main/java/com/example/meepmeeptesting/Juice-CENTERSTAGE-Dark.png")); }
        catch (IOException e) {}

        meepMeep.setBackground(img)
//        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(myBotCyclesUnsafeSide)
                .addEntity(myBotCyclesSafeRed)
                //.addEntity(myBotCycleSafeBlue)
//                .addEntity(myBotCyclesSafeOpti)
                .start();
    }
}