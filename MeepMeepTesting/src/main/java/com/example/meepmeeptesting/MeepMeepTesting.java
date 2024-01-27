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

        RoadRunnerBotEntity myBotCyclesSafeRed = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(55, 55, Math.toRadians(360), Math.toRadians(360), 13.28)
                .setDimensions(15, 17)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(62, -34, Math.toRadians(0)))
                                        .setReversed(true)
                                        .splineToLinearHeading(new Pose2d(34, -32, Math.toRadians(305)), Math.toRadians(150))
                                        .forward(12)
                                        .turn(Math.toRadians(-35))
                                        .strafeLeft(6)
                                        //.splineToLinearHeading(new Pose2d(-48, -40, Math.toRadians(235)), Math.toRadians(30))
                                        //.splineToLinearHeading(new Pose2d(-57, -40, Math.toRadians(-90)), Math.toRadians(-90))
                                        //.setReversed(false)
                                        //.splineToConstantHeading(new Vector2d(-57, -25), Math.toRadians(90))
                                        .splineToConstantHeading(new Vector2d(57, 10), Math.toRadians(90))
                                        .splineToConstantHeading(new Vector2d(31, 50), Math.toRadians(90))
                                        .addTemporalMarker(0, () -> {
                                        })
                                        .addTemporalMarker(2, () -> {
                                        })
                                        .addTemporalMarker(3.5, () -> {
                                        })
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
//                .addEntity(myBotCyclesSafeOpti)
                .start();
    }
}