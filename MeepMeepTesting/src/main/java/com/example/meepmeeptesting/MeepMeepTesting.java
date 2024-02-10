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
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(62, 13, Math.toRadians(0)))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(38, 11.7, Math.toRadians(55)), Math.toRadians(30))

                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(30, 49, Math.toRadians(270)), Math.toRadians(90))
                        .addTemporalMarker(0, () -> {
                            //this.robot.intake.setAngle(120);
                        })
                        .addTemporalMarker(1.5, () -> {
                            //robot.autoPreloadDepositPreset();
                        })
                        .addTemporalMarker(2.3, () -> {
                            //robot.smartClawOpen();
                        })
                        .waitSeconds(0.8)
                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(-90))
                                .splineToConstantHeading(new Vector2d(13, -56.1), Math.toRadians(-90))
//                .addTemporalMarker(2, () -> {
//                    robot.autoIntake(3, 170);
//                })
                                .setReversed(true)
                                .addTemporalMarker(2.5, () -> {
                                    //startIntake();
                                })
                                .strafeLeft(4)
                                .forward(2)
                                .strafeRight(8)
                                .back(2)
                                .addTemporalMarker(4.5, () -> {
                                    //intake.setAngle(90);
                                })
                                .addTemporalMarker(4.5, () -> {
                                    //intake.reverseIntake();
                                })
                                .addTemporalMarker(4.7, () -> {
                                    //stopIntake();
                                })
                                .waitSeconds(0.2)
                                .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(30, 48), Math.toRadians(90))
                                .addTemporalMarker(6.5, () -> {
                                    //startAutoIntake();
                                    //claw.setClawOpen();
                                })
                                .addTemporalMarker(7, () -> {
                                    //stopIntake();
                                })
                                .addTemporalMarker(8.2, ()-> {
                                    //autoCycleDepositPreset();
                                })
                                .addTemporalMarker(9.5, ()-> {
                                    //smartClawOpen();
                                })
                                .waitSeconds(1)
                        .build()
                );


        RoadRunnerBotEntity myBotCyclesSafeRed = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(15, 17)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-62, -34, Math.toRadians(180)))
                                        .setReversed(true)
                                        .addTemporalMarker(0, () -> {
                                            //this.robot.farPos();
                                        })
                                        .splineTo(new Vector2d(-38, -27), Math.toRadians(45))
                                        .forward(17)
                                        .turn(Math.toRadians(45))
                                        .strafeRight(10)
                                        .back(20)
                                        .splineToConstantHeading(new Vector2d(-59, 20), Math.toRadians(90))
                                        .splineToConstantHeading(new Vector2d(-38, 49), Math.toRadians(90))
                                        .addTemporalMarker(0, () -> {
                                            //this.robot.intake.setAngle(120);
                                        })
                                        .addTemporalMarker(3, () -> {
                                            //robot.autoPreloadDepositPreset();
                                        })
                                        .addTemporalMarker(4, () -> {
                                            //robot.smartClawOpen();
                                        })
                                        .waitSeconds(2)
                                        .strafeRight(20)
                                        .back(10)
                                        .build()

                );

        RoadRunnerBotEntity myBotRelocal = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(15, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(32.5, 49.5, Math.toRadians(90)))
                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(-90))
                                .splineToConstantHeading(new Vector2d(14, -56), Math.toRadians(-90))
//                .addTemporalMarker(2, () -> {
//                    robot.autoIntake(3, 170);
//                })
                                .setReversed(true)
                                .addTemporalMarker(2.9, () -> {
//                                    robot.intake.setAngle(182);
//                                    robot.intake.intakeMotor.setSpeed((float)0.3);
                                })
                                .back(9)
                                .forward(9)
                                .addTemporalMarker(4.5, () -> {
//                                    robot.startIntake();
                                })
//                .addTemporalMarker(4.5, () -> {
//                    robot.intake.reverseIntake();
//                })
                                .addTemporalMarker(5.5, () -> {
//                                    robot.stopIntake();
                                })
                                .waitSeconds(2)
                                .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(30, 37.7), Math.toRadians(90))
                                .waitSeconds(1)
                                .splineToConstantHeading(new Vector2d(30, 47.5), Math.toRadians(90))
                                .addTemporalMarker(7, () -> {
//                                    robot.startIntake();
//                                    robot.claw.setClawOpen();
                                })
                                .addTemporalMarker(8, () -> {
//                                    robot.stopIntake();
                                })
                                .addTemporalMarker(10, () -> {
//                                    Pose2d newPose = robot.cv.relocalizeUsingBackdrop(drive.getPoseEstimate());
//                                    drive.setPoseEstimate(newPose);
                                })
                                .addTemporalMarker(10, ()-> {
//                                    robot.slides.runToPosition(50);
//                                    robot.autoCycleDepositPreset();
                                })
                                .addTemporalMarker(12.7, ()-> {
//                                    robot.smartClawOpen();
                                })
                                .waitSeconds(2)
                                .build()

                );


        Image img = null;
//        try { img = ImageIO.read(new File("/Users/zhimi/Downloads/field.png")); }
        try { img = ImageIO.read(new File("/Users/siddharth/dev/Juice/CenterStage/MeepMeepTesting/src/main/java/com/example/meepmeeptesting/Juice-CENTERSTAGE-Dark.png")); }
        catch (IOException e) {}

        meepMeep.setBackground(img)
//        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(myBotCyclesUnsafeSide)
//                .addEntity(myBotCyclesSafeRed)
//                .addEntity(myBotCycleSafeBlue)
//                .addEntity(myBotCyclesSafeOpti)
                .addEntity(myBotRelocal)
                .start();
    }
}