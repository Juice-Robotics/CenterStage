package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.AllianceColor;
import org.firstinspires.ftc.teamcode.subsystems.vision.TeamElementCVProcessor;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;

@Config
@Autonomous(group = "drive")

public class BlueLeftMain extends LinearOpMode {
    Robot robot;
    VisionPortal visionPortal;
    TeamElementCVProcessor teamElementProcessor;
    TeamElementCVProcessor.Location propLocation = TeamElementCVProcessor.Location.UNFOUND;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot = new Robot(hardwareMap, true);
        Pose2d startPose = new Pose2d(in(92), in(165), rad(90));
        drive.setPoseEstimate(startPose);

        teamElementProcessor = new TeamElementCVProcessor(
                () -> 100, // these are lambda methods, in case we want to change them while the match is running, for us to tune them or something
                () -> 213, // the left dividing line, in this case the left third of the frame
                () -> 426, // the left dividing line, in this case the right third of the frame,
                telemetry,
                AllianceColor.BLUE);
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
                .addProcessor(teamElementProcessor)
                .build();

        // PRELOAD PATHS
        TrajectorySequence preloadSpikeLeft = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-45, 20), Math.toRadians(15))
                .strafeLeft(10)
                .build();
        TrajectorySequence preloadBackdropLeft = drive.trajectorySequenceBuilder(preloadSpikeLeft.end())
                .setReversed(false)
                .splineTo(new Vector2d(-29, 49), Math.toRadians(90))
                .build();
        TrajectorySequence leftToStack1 = drive.trajectorySequenceBuilder(preloadBackdropLeft.end())
                .setReversed(true)
                .splineTo(new Vector2d(-10, 20), Math.toRadians(-90))
                .splineTo(new Vector2d(-11, -61), Math.toRadians(-90))
                .build();

        TrajectorySequence preloadSpikeCenter = drive.trajectorySequenceBuilder(startPose)
                .forward(20)
                .back(10)
                .build();
        TrajectorySequence preloadBackdropCenter = drive.trajectorySequenceBuilder(preloadSpikeCenter.end())
                .setReversed(false)
                .splineTo(new Vector2d(-36, 49), Math.toRadians(90))
                .build();
        TrajectorySequence centerToStack1 = drive.trajectorySequenceBuilder(preloadBackdropCenter.end())
                .setReversed(true)
                .splineTo(new Vector2d(-10, 20), Math.toRadians(-90))
                .splineTo(new Vector2d(-11, -61), Math.toRadians(-90))
                .build();

        TrajectorySequence preloadSpikeRight = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-43, 8), Math.toRadians(-15))
                .back(10)
                .build();
        TrajectorySequence preloadBackdropRight = drive.trajectorySequenceBuilder(preloadSpikeRight.end())
                .setReversed(false)
                .splineTo(new Vector2d(-40, 49), Math.toRadians(90))
                .build();
        TrajectorySequence rightToStack1 = drive.trajectorySequenceBuilder(preloadBackdropRight.end())
                .setReversed(true)
                .splineTo(new Vector2d(-10, 20), Math.toRadians(-90))
                .splineTo(new Vector2d(-11, -61), Math.toRadians(-90))
                .build();

        // SHARED PATHS
        TrajectorySequence stackToBackdrop1 = drive.trajectorySequenceBuilder(leftToStack1.end())
                .setReversed(false)
                .splineTo(new Vector2d(-10, 20), Math.toRadians(90))
                .splineTo(new Vector2d(-36, 49), Math.toRadians(90))
                .build();

        TrajectorySequence backdropToStack2 = drive.trajectorySequenceBuilder(stackToBackdrop1.end())
                .setReversed(true)
                .splineTo(new Vector2d(-10, 20), Math.toRadians(-90))
                .splineTo(new Vector2d(-11, -61), Math.toRadians(-90))
                .build();
        TrajectorySequence stackToBackdrop2 = drive.trajectorySequenceBuilder(backdropToStack2.end())
                .setReversed(false)
                .splineTo(new Vector2d(-10, 20), Math.toRadians(90))
                .splineTo(new Vector2d(-36, 49), Math.toRadians(90))
                .build();

        TrajectorySequence backdropToStack3 = drive.trajectorySequenceBuilder(stackToBackdrop2.end())
                .setReversed(true)
                .splineTo(new Vector2d(-10, 20), Math.toRadians(-90))
                .splineTo(new Vector2d(-11, -61), Math.toRadians(-90))
                .build();
        TrajectorySequence stackToBackdrop3 = drive.trajectorySequenceBuilder(backdropToStack3.end())
                .setReversed(false)
                .splineTo(new Vector2d(-10, 20), Math.toRadians(90))
                .splineTo(new Vector2d(-36, 49), Math.toRadians(90))
                .build();

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            TeamElementCVProcessor.Location reading = teamElementProcessor.getLocation();
            telemetry.addData("Camera State", visionPortal.getCameraState());

            if (reading == TeamElementCVProcessor.Location.UNFOUND) {
                telemetry.addLine("Team Element Location: <b>NOT FOUND</b>");
            } else {
                telemetry.addData("Team Element Location", reading);
            }

            telemetry.update();
        }


        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        // shuts down the camera once the match starts, we dont need to look any more
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }

        propLocation = teamElementProcessor.getLocation();

        // if it is UNFOUND, you can manually set it to any of the other positions to guess
        if (propLocation == TeamElementCVProcessor.Location.UNFOUND) {
            propLocation = TeamElementCVProcessor.Location.CENTER;
        }

        waitForStart();

        if (isStopRequested()) return;

        switch (propLocation) {
            case CENTER:
                drive.followTrajectorySequence(preloadSpikeCenter);
                drive.followTrajectorySequence(preloadBackdropCenter);
                drive.followTrajectorySequence(centerToStack1);
                break;
            case LEFT:
                drive.followTrajectorySequence(preloadSpikeLeft);
                drive.followTrajectorySequence(preloadBackdropLeft);
                drive.followTrajectorySequence(leftToStack1);
                break;
            case RIGHT:
                drive.followTrajectorySequence(preloadSpikeRight);
                drive.followTrajectorySequence(preloadBackdropCenter);
                drive.followTrajectorySequence(rightToStack1);
                break;
        }

        drive.followTrajectorySequence(stackToBackdrop1);
        drive.followTrajectorySequence(backdropToStack2);
        drive.followTrajectorySequence(stackToBackdrop2);
        drive.followTrajectorySequence(backdropToStack3);
        drive.followTrajectorySequence(stackToBackdrop3);

        while (!isStopRequested() && opModeIsActive()) ;
    }

    public static double rad(double degrees) {
        return Math.toRadians(degrees);
    }

    public static double in(double centimeters) {
        return centimeters * 0.3837008;
    }
}
