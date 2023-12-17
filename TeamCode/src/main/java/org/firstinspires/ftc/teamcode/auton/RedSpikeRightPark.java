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
import org.firstinspires.ftc.teamcode.lib.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.vision.TeamElementCVProcessor;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(group = "drive")

public class RedSpikeRightPark extends LinearOpMode {
    Robot robot;
    VisionPortal visionPortal;
    TeamElementCVProcessor teamElementProcessor;
    TeamElementCVProcessor.Location propLocation = TeamElementCVProcessor.Location.UNFOUND;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot = new Robot(hardwareMap, true);
        Pose2d startPose = new Pose2d(59, 12, Math.PI);
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
                .addTemporalMarker(0, () -> {
                    robot.runToAutoSpikePreset();
                })
                .splineTo(new Vector2d(45, 6), Math.toRadians(195))
                .waitSeconds(1)
                .addTemporalMarker(1, () -> {
                    robot.claw.setClawOpen();
                    robot.intakePreset();
                    robot.claw.setClawClose();
                })
                .build();
        TrajectorySequence preloadParkLeft = drive.trajectorySequenceBuilder(preloadSpikeLeft.end())
                .setReversed(false)
                .back(0.5)
                .splineTo(new Vector2d(60, 60), Math.toRadians(90))
                .build();

        TrajectorySequence preloadSpikeCenter = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    robot.runToAutoSpikePreset();
                })
                .forward(20)
                .waitSeconds(1)
                .addTemporalMarker(1, () -> {
                    robot.claw.setClawOpen();
                    robot.intakePreset();
                    robot.claw.setClawClose();
                })
                .back(10)
                .build();
        TrajectorySequence preloadParkCenter = drive.trajectorySequenceBuilder(preloadSpikeCenter.end())
                .setReversed(false)
                .back(0.1)
                .splineTo(new Vector2d(58, 60), Math.toRadians(90))
                .build();

        TrajectorySequence preloadSpikeRight = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, () -> {
                    robot.runToAutoSpikePreset();
                })
                .splineTo(new Vector2d(43, 20), Math.toRadians(165))
                .waitSeconds(1)
                .addTemporalMarker(1, () -> {
                    robot.claw.setClawOpen();
                    robot.intakePreset();
                    robot.claw.setClawClose();
                })
                .back(10)
                .build();
        TrajectorySequence preloadParkRight = drive.trajectorySequenceBuilder(preloadSpikeRight.end())
                .setReversed(false)
                .splineTo(new Vector2d(60, 60), Math.toRadians(90))
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
                drive.followTrajectorySequence(preloadParkCenter);
                break;
            case LEFT:
                drive.followTrajectorySequence(preloadSpikeLeft);
                drive.followTrajectorySequence(preloadParkLeft);
                break;
            case RIGHT:
                drive.followTrajectorySequence(preloadSpikeRight);
                drive.followTrajectorySequence(preloadParkRight);
                break;
        }


        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = drive.getPoseEstimate();

        while (!isStopRequested() && opModeIsActive()) ;
    }

    public static double rad(double degrees) {
        return Math.toRadians(degrees);
    }

    public static double in(double centimeters) {
        return centimeters * 0.3837008;
    }
}
