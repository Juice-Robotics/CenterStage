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
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(62, 13, Math.toRadians(180)))
                        .setReversed(true)
                        .splineTo(new Vector2d(39, 13), Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(48, 13, Math.toRadians(0)), Math.toRadians(0))
                        .splineTo(new Vector2d(33, 50), Math.toRadians(90))
                        .build());


        RoadRunnerBotEntity myBotCyclesSafeRed = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(15, 17)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-62, 13, Math.toRadians(180)))
                                        .setReversed(true)
                                        .splineTo(new Vector2d(-41, 13), Math.toRadians(0))
                                        .splineToLinearHeading(new Pose2d(-51, 13, Math.toRadians(180)), Math.toRadians(180))
                                        .setReversed(true)
                                        .splineTo(new Vector2d(-33, 52), Math.toRadians(90))
                                        .addTemporalMarker(0, () -> {
                                            //this.//intake.setAngle(120);
                                        })
                                        .addTemporalMarker(1.1, () -> {
                                            //autoPreloadDepositPreset();
                                        })
                                        .addTemporalMarker(1.95, () -> {
                                            //smartClawOpen();
                                        })
                                        .waitSeconds(1.5)
                                        .setReversed(false)
                                        .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(-90))
                                        .splineToConstantHeading(new Vector2d(-16, -56.5), Math.toRadians(-90))
//                .addTemporalMarker(2, () -> {
//                    robot.autoIntake(3, 170);
//                })
                                        .setReversed(true)
                                        .addTemporalMarker(3, () -> {
                                            //robot.intake.setAngle(188);
                                        })
                                        .addTemporalMarker(4.7, () ->{
                                            //robot.intake.autoStartIntake();
                                        })
                                        .splineToConstantHeading(new Vector2d(-26, -56.5), Math.toRadians(-90))
                                        .splineToConstantHeading(new Vector2d(-26, -50.5), Math.toRadians(-90))
                                        .splineToConstantHeading(new Vector2d(-26, -55.5), Math.toRadians(-90))
                                        //.strafeRight(8)
                                        .addTemporalMarker(6, () -> {
                                            //robot.stopIntake();
                                            //robot.intake.setAngle(90);
                                        })
//                .addTemporalMarker(5.1, () -> {
//                    robot.intake.reverseIntake();
//                })

                                        .waitSeconds(1)
                                        .splineToConstantHeading(new Vector2d(-8, -15), Math.toRadians(90))
                                        .splineToConstantHeading(new Vector2d(-8, 20), Math.toRadians(90))
                                        .splineToConstantHeading(new Vector2d(-28, 51.5), Math.toRadians(90))
                                        .addTemporalMarker(8, () -> {
                                            //robot.autoCycleDepositPreset();
                                        })
                                        .addTemporalMarker(9.5, () -> {
                                            //robot.smartClawOpen();
                                        })
                                        .waitSeconds(1.5)
                                        //
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