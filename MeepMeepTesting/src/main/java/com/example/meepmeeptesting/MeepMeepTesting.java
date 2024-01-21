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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(14, 17)
                .followTrajectorySequence(drive ->
                                drive.trajectorySequenceBuilder(new Pose2d(62, 13, Math.toRadians(0)))
                                        .setReversed(true)
                                        .splineTo(new Vector2d(38, 25), Math.toRadians(180))
                                        .setReversed(false)
                                        .splineToConstantHeading(new Vector2d(44, 25), Math.toRadians(180))


//                                        .splineToLinearHeading(new Pose2d(43, 42, Math.toRadians(270)), Math.toRadians(90))
                                        .splineToLinearHeading(new Pose2d(42, 50, Math.toRadians(270)), Math.toRadians(90))
                                        .addTemporalMarker(0, () -> {
//                                            this.robot.intake.setAngle(120);
                                        })
                                        .addTemporalMarker(2, () -> {
//                                            robot.autoPreloadDepositPreset();
                                        })
                                        .addTemporalMarker(3.5, () -> {
//                                            robot.smartClawOpen();
                                        })
                                        .waitSeconds(3)
                                        .build()
                );


        Image img = null;
//        try { img = ImageIO.read(new File("/Users/huntert/Downloads/Juice-CENTERSTAGE-Dark.png")); }
        try { img = ImageIO.read(new File("/Users/siddharth/dev/Juice/CenterStage/MeepMeepTesting/src/main/java/com/example/meepmeeptesting/Juice-CENTERSTAGE-Dark.png")); }
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