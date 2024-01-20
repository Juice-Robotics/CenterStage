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
                                drive.trajectorySequenceBuilder(new Pose2d(-33, 52, Math.toRadians(-90)))
                                        .setReversed(false)
                                        .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(-90))
                                        .splineToConstantHeading(new Vector2d(-16, -57), Math.toRadians(-90))
//                .addTemporalMarker(2, () -> {
//                    robot.autoIntake(3, 170);
//                })
                                        .setReversed(true)
                                        .splineToConstantHeading(new Vector2d(-26, -57), Math.toRadians(-90))
                                        .splineToConstantHeading(new Vector2d(-26, -51), Math.toRadians(-90))
                                        .splineToConstantHeading(new Vector2d(-26, -56), Math.toRadians(-90))
                                        .build()
                );


        Image img = null;
        try { img = ImageIO.read(new File("/Users/huntert/Downloads/Juice-CENTERSTAGE-Dark.png")); }
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