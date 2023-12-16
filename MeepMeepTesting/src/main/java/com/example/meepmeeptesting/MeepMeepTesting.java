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

        RoadRunnerBotEntity myBot2Plus0 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-59, 12, 0))
                                .forward(20)
                                .back(10)
                                .setReversed(false)
                                .splineTo(new Vector2d(-36, 49), Math.toRadians(90))
                                .strafeLeft(25)
                                .build()
                );

        RoadRunnerBotEntity myBotCyclesObv = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-59, 12, 0))
                                .forward(20)
                                .back(10)
                                .setReversed(false)
                                .splineTo(new Vector2d(-36, 49), Math.toRadians(90))
                                .setReversed(true)
                                .splineTo(new Vector2d(-35, -60), Math.toRadians(-90))
                                .setReversed(false)
                                .splineTo(new Vector2d(-36, 49), Math.toRadians(90))
                                .build()
                );

        RoadRunnerBotEntity myBotCyclesSafe = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14, 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-59, 12, 0))
                                //CENTER
//                                .forward(20)
//                                .back(10)
//                                .setReversed(false)
//                                .splineTo(new Vector2d(-36, 49), Math.toRadians(90))

                                // RIGHT
//                                .splineTo(new Vector2d(-43, 8), Math.toRadians(-15))
//                                .back(10)
//                                .setReversed(false)
//                                .splineTo(new Vector2d(-40, 49), Math.toRadians(90))

                                //LEFT
                                .splineTo(new Vector2d(-45, 20), Math.toRadians(15))
                                .strafeLeft(10)
                                .setReversed(false)
                                .splineTo(new Vector2d(-29, 49), Math.toRadians(90))


                                .setReversed(true)
                                .splineTo(new Vector2d(-10, 20), Math.toRadians(-90))
                                .splineTo(new Vector2d(-11, -61), Math.toRadians(-90))
                                .setReversed(false)
                                .splineTo(new Vector2d(-10, 20), Math.toRadians(90))
                                .splineTo(new Vector2d(-36, 49), Math.toRadians(90))

                                .setReversed(true)
                                .splineTo(new Vector2d(-10, 20), Math.toRadians(-90))
                                .splineTo(new Vector2d(-11, -61), Math.toRadians(-90))
                                .setReversed(false)
                                .splineTo(new Vector2d(-10, 20), Math.toRadians(90))
                                .splineTo(new Vector2d(-36, 49), Math.toRadians(90))

                                .setReversed(true)
                                .splineTo(new Vector2d(-10, 20), Math.toRadians(-90))
                                .splineTo(new Vector2d(-11, -61), Math.toRadians(-90))
                                .setReversed(false)
                                .splineTo(new Vector2d(-10, 20), Math.toRadians(90))
                                .splineTo(new Vector2d(-36, 49), Math.toRadians(90))

                                .build()
                );

        RoadRunnerBotEntity myBotCyclesSafeOpti = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(50, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14, 17)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-59, 12, 0))
                                        //CENTER
                                .forward(20)
                                .back(10)
                                .setReversed(false)
                                .splineTo(new Vector2d(-36, 49), Math.toRadians(90))

                                        // RIGHT
//                                .splineTo(new Vector2d(-43, 8), Math.toRadians(-15))
//                                .back(10)
//                                .setReversed(false)
//                                .splineTo(new Vector2d(-40, 49), Math.toRadians(90))

                                        //LEFT
//                                        .splineTo(new Vector2d(-45, 20), Math.toRadians(15))
//                                        .strafeLeft(10)
//                                        .setReversed(false)
//                                        .splineTo(new Vector2d(-29, 49), Math.toRadians(90))


                                        .setReversed(true)
                                        .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(-90))
                                        .splineToConstantHeading(new Vector2d(-11, -61), Math.toRadians(-90))
                                        .setReversed(false)
                                        .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(90))
                                        .splineToConstantHeading(new Vector2d(-36, 49), Math.toRadians(90))

                                        .setReversed(true)
                                        .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(-90))
                                        .splineToConstantHeading(new Vector2d(-11, -61), Math.toRadians(-90))
                                        .setReversed(false)
                                        .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(90))
                                        .splineToConstantHeading(new Vector2d(-36, 49), Math.toRadians(90))

                                        .setReversed(true)
                                        .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(-90))
                                        .splineToConstantHeading(new Vector2d(-11, -61), Math.toRadians(-90))
                                        .setReversed(false)
                                        .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(90))
                                        .splineToConstantHeading(new Vector2d(-36, 49), Math.toRadians(90))

                                        .build()
                );

        RoadRunnerBotEntity myBotCyclesSafeRed = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14, 17)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(59, 12, Math.toRadians(-180)))
                                        //CENTER
                                .forward(20)
                                .back(10)
                                .setReversed(false)
                                .splineTo(new Vector2d(36, 49), Math.toRadians(90))

                                        // LEFT
//                                .splineTo(new Vector2d(43, 8), Math.toRadians(200))
//                                .back(10)
//                                .setReversed(false)
//                                .splineTo(new Vector2d(29, 49), Math.toRadians(90))

                                        //RIGHT
//                                        .splineTo(new Vector2d(45, 20), Math.toRadians(165))
//                                        .strafeRight(10)
//                                        .setReversed(false)
//                                        .splineTo(new Vector2d(40, 49), Math.toRadians(90))


                                        .setReversed(true)
                                        .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(270))
                                        .splineToConstantHeading(new Vector2d(11, -61), Math.toRadians(270))
                                        .setReversed(false)
                                        .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(90))
                                        .splineToConstantHeading(new Vector2d(36, 49), Math.toRadians(90))

                                        .setReversed(true)
                                        .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(270))
                                        .splineToConstantHeading(new Vector2d(11, -61), Math.toRadians(270))
                                        .setReversed(false)
                                        .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(90))
                                        .splineToConstantHeading(new Vector2d(36, 49), Math.toRadians(90))

                                        .setReversed(true)
                                        .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(270))
                                        .splineToConstantHeading(new Vector2d(11, -61), Math.toRadians(270))
                                        .setReversed(false)
                                        .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(90))
                                        .splineToConstantHeading(new Vector2d(36, 49), Math.toRadians(90))

                                        .build()
                );

        RoadRunnerBotEntity myBotCyclesUnsafeSide = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14, 17)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(-59, -35, Math.toRadians(180)))
                                        //CENTER
                                        .back(25)
                                        .forward(7)
                                        .setReversed(false)
                                        .splineTo(new Vector2d(-35, -60), Math.toRadians(270))
                                        .setReversed(true)
                                        .splineTo(new Vector2d(-10, -42.5), Math.toRadians(90))
                                        .splineTo(new Vector2d(-10, 20), Math.toRadians(90))
                                        .splineTo(new Vector2d(-29, 49), Math.toRadians(90))

                                        // RIGHT
//                                        .splineTo(new Vector2d(-43, -42), Math.toRadians(-15))
//                                        .back(10)
//                                        .setReversed(false)
//                                        .lineToSplineHeading(new Pose2d(-10, -31, Math.toRadians(90)))
//                                        .setReversed(true)
//                                        .splineTo(new Vector2d(-11, -60), Math.toRadians(270))
//                                        .setReversed(false)
//                                        .splineTo(new Vector2d(-10, -42.5), Math.toRadians(90))
//                                        .splineTo(new Vector2d(-10, 20), Math.toRadians(90))
//                                        .splineTo(new Vector2d(-29, 49), Math.toRadians(90))

                                        //RIGHT sussy
//                                        .back(26)
//                                        .turn(Math.toRadians(-90))
//                                        .strafeRight(21)
//                                        .turn(Math.toRadians(-180))
//                                        .forward(20)
////                                        .back(40)
//                                        .setReversed(true)
//                                        .splineTo(new Vector2d(-10, 20), Math.toRadians(90))
//                                        .splineTo(new Vector2d(-29, 49), Math.toRadians(90))

                                        //LEFT
//                                        .setReversed(true)
//                                        .splineTo(new Vector2d(-36, -34), Math.toRadians(55))
//                                        .setReversed(false)
//                                        .splineTo(new Vector2d(-35, -60), Math.toRadians(270))
//                                        .setReversed(true)
//                                        .splineTo(new Vector2d(-10, -42.5), Math.toRadians(90))
//                                        .splineTo(new Vector2d(-10, 20), Math.toRadians(90))
//                                        .splineTo(new Vector2d(-29, 49), Math.toRadians(90))

                                        .setReversed(false)
                                        .splineTo(new Vector2d(-10, 20), Math.toRadians(-90))
                                        .splineTo(new Vector2d(-11, -61), Math.toRadians(-90))
                                        .setReversed(true)
                                        .splineTo(new Vector2d(-10, 20), Math.toRadians(90))
                                        .splineTo(new Vector2d(-36, 49), Math.toRadians(90))

                                        .setReversed(false)
                                        .splineTo(new Vector2d(-10, 20), Math.toRadians(-90))
                                        .splineTo(new Vector2d(-11, -61), Math.toRadians(-90))
                                        .setReversed(true)
                                        .splineTo(new Vector2d(-10, 20), Math.toRadians(90))
                                        .splineTo(new Vector2d(-36, 49), Math.toRadians(90))

//                                        .setReversed(false)
//                                        .splineTo(new Vector2d(-10, 20), Math.toRadians(-90))
//                                        .splineTo(new Vector2d(-11, -61), Math.toRadians(-90))
//                                        .setReversed(true)
//                                        .splineTo(new Vector2d(-10, 20), Math.toRadians(90))
//                                        .splineTo(new Vector2d(-36, 49), Math.toRadians(90))



                                        .build()
                );

        Image img = null;
        try { img = ImageIO.read(new File("/Users/siddharth/dev/Juice/CenterStage/MeepMeepTesting/src/main/java/com/example/meepmeeptesting/Juice-CENTERSTAGE-Dark.png")); }
        catch (IOException e) {}

        meepMeep.setBackground(img)
//        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(myBotCyclesUnsafeSide)
//                .addEntity(myBotCyclesSafeRed)
                .addEntity(myBotCyclesSafeOpti)
                .start();
    }
}