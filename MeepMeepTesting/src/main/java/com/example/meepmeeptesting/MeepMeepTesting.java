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
        double spikeTimel = 5.91;
        double spikeTimell = 5.12;
        double cycletime = 8.96;
        double spikeTimec = 0;
        double spikeTimer = 0;
        RoadRunnerBotEntity myBotCycleSafeBlue = new DefaultBotBuilder(meepMeep)
                .setConstraints(55, 55, Math.toRadians(360), Math.toRadians(360), 13.28)
                .setDimensions(15, 17)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-62, -34, Math.toRadians(180)))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-22, -47, Math.toRadians(180)), Math.toRadians(180))
                        .waitSeconds(1)

                        .addTemporalMarker(1.8, () -> {
                            //release pixel from intake
                        })
                        //stack
                        .splineToLinearHeading(new Pose2d(-12, -57, Math.toRadians(-90)), Math.toRadians(-90))
                        .waitSeconds(1.5)
                        //break
                        .splineToConstantHeading(new Vector2d(-10, 25), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-42, 51), Math.toRadians(90))

                                .waitSeconds(1)
                        //cycles
                        .setReversed(false)
                        .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(-13, -55.8), Math.toRadians(-90))
//                .addTemporalMarker(2, () -> {
//                    robot.autoIntake(3, 170);
//                })
                        .setReversed(true)
                        .addTemporalMarker(3.2, () -> {
                            //startIntake();
                        })
                        .addTemporalMarker(4.7, () -> {
                            //stopIntake();
                        })
                        .waitSeconds(1)
                        .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-30, 47.7), Math.toRadians(90))

                        .addTemporalMarker(5, () -> {
                            //startAutoIntake();
                            //claw.setClawOpen();
                        })
                        .addTemporalMarker(6, () -> {
                            //stopIntake();
                        })
                        .addTemporalMarker(7.3, ()-> {
                            //autoCycleDepositPreset();
                        })
                        .addTemporalMarker(8, ()-> {
                            //smartClawOpen();
                        })
                        .waitSeconds(1.2)
                        .build()
                );


        RoadRunnerBotEntity myBotCyclesSafeRed = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(55, 55, Math.toRadians(360), Math.toRadians(360), 13.28)
                .setDimensions(15, 17)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(62, -34, Math.toRadians(0)))
                                        .setReversed(true)
                                        .splineToLinearHeading(new Pose2d(22, -47, Math.toRadians(0)), Math.toRadians(0))
                                        .waitSeconds(1)

                                        .addTemporalMarker(1.8, () -> {
                                            //release pixel from intake
                                        })
                                        .splineToLinearHeading(new Pose2d(12, -57, Math.toRadians(-90)), Math.toRadians(-90))
                                        .waitSeconds(1.5)
                                        //backdrop
                                        .splineToConstantHeading(new Vector2d(10, 25), Math.toRadians(90))
                                        .splineToConstantHeading(new Vector2d(35, 47.7), Math.toRadians(90))

                                        //cycles
                                        .setReversed(false)
                                        .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(-90))
                                        .splineToConstantHeading(new Vector2d(12, -55.8), Math.toRadians(-90))
//                .addTemporalMarker(2, () -> {
//                    robot.autoIntake(3, 170);
//                })
                                        .setReversed(true)
                                        .addTemporalMarker(3.2, () -> {
                                 //           robot.startIntake();
                                        })
                                        .addTemporalMarker(4.7, () -> {
                                 //           robot.stopIntake();
                                        })
                                        .waitSeconds(1)
                                        .splineToConstantHeading(new Vector2d(10, 25), Math.toRadians(90))
                                        .splineToConstantHeading(new Vector2d(30, 47.7), Math.toRadians(90))

                                        .addTemporalMarker(5, () -> {
                                   //         robot.startAutoIntake();
                                   //         robot.claw.setClawOpen();
                                        })
                                        .addTemporalMarker(6, () -> {
                                    //        robot.stopIntake();
                                        })
                                        .addTemporalMarker(7.3, ()-> {
                                      //      robot.autoCycleDepositPreset();
                                        })
                                        .addTemporalMarker(8, ()-> {
                                         //   robot.smartClawOpen();
                                        })
                                        .waitSeconds(1.2)
                                        .strafeLeft(29)
                                        .back(12)
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
               // .addEntity(myBotCyclesSafeRed)
                .addEntity(myBotCycleSafeBlue)
//                .addEntity(myBotCyclesSafeOpti)
                .start();
    }
}