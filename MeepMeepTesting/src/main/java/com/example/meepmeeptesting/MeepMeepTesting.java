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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(15, 17)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(62, -34, Math.toRadians(0)))
                                        .setReversed(true)
                                        .splineTo(new Vector2d(38, -47), Math.toRadians(180))
                                        .forward(17)
                                        .turn(Math.toRadians(-90))
                                        .setReversed(true)
                                        //.setReversed(true)
                                        .splineToLinearHeading(new Pose2d(58, -8, Math.toRadians(-90)), Math.toRadians(90))
                                        .splineToLinearHeading(new Pose2d(30, 52, Math.toRadians(-90)), Math.toRadians(90))
                                        .build()

                );


        Image img = null;
        try { img = ImageIO.read(new File("/Users/zhimi/Downloads/field.png")); }
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