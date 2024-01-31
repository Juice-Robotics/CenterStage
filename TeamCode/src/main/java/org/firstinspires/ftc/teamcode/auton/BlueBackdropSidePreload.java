package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.lib.AllianceColor;
import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.vision.YoinkP2Pipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;

@Config
@Autonomous(group = "drive")

public class BlueBackdropSidePreload extends LinearOpMode {
    Robot robot;
    private VisionPortal visionPortal;
    private YoinkP2Pipeline colourMassDetectionProcessor;
    AprilTagProcessor processor;

    @Override
    public void runOpMode() throws InterruptedException {
        Scalar lower = new Scalar(103, 120, 50); // the lower hsv threshold for your detection
        Scalar upper = new Scalar(130, 255, 250); // the upper hsv threshold for your detection
        double minArea = 3000; // the minimum area for the detection to consider for your prop

        colourMassDetectionProcessor = new YoinkP2Pipeline(
                lower,
                upper,
                () -> minArea, // these are lambda methods, in case we want to change them while the match is running, for us to tune them or something
                () -> 213, // the left dividing line, in this case the left third of the frame
                () -> 426 // the left dividing line, in this case the right third of the frame
        );
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
                .addProcessor(colourMassDetectionProcessor)
                .build();

        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        robot = new Robot(hardwareMap, true);
        Pose2d startPose = new Pose2d(-62, 13, Math.toRadians(180));
        robot.initPos();

        drive.setPoseEstimate(startPose);

        // PRELOAD PATHS
        TrajectorySequence preloadSpikeLeft = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(-38, 21.5), Math.toRadians(0))
                .forward(15)
                .build();

        TrajectorySequence preloadBackdropLeft = drive.trajectorySequenceBuilder(preloadSpikeLeft.end())
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-44, 51, Math.toRadians(270)), Math.toRadians(90))
                .addTemporalMarker(0, () -> {
                    this.robot.intake.setAngle(120);
                })
                .addTemporalMarker(1.1, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(2.1, () -> {
                    robot.smartClawOpen();
                })
                .addTemporalMarker(3, () -> {
                    robot.slides.runToPosition(0);
                })
                .waitSeconds(1)
                .strafeRight(18)
                .back(5)
                .waitSeconds(3)
                .build();

        TrajectorySequence preloadSpikeCenter = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(-40, 13), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-48, 13, Math.toRadians(180)), Math.toRadians(0))
                .strafeRight(4)
                .build();

        TrajectorySequence preloadBackdropCenter = drive.trajectorySequenceBuilder(preloadSpikeCenter.end())
                .setReversed(true)
                .splineTo(new Vector2d(-37, 52.5), Math.toRadians(90))
                .addTemporalMarker(0, () -> {
                    this.robot.intake.setAngle(120);
                })
                .addTemporalMarker(1.1, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(2.1, () -> {
                    robot.smartClawOpen();
                })
                .addTemporalMarker(3, () -> {
                    robot.slides.runToPosition(0);
                })
                .waitSeconds(1)
                .strafeRight(24)
                .back(5)
                .waitSeconds(3)
                .build();

        TrajectorySequence preloadSpikeRight = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-36.5, 5, Math.toRadians(145)), Math.toRadians(-32))
                .forward(5)
                .build();

        TrajectorySequence preloadBackdropRight = drive.trajectorySequenceBuilder(preloadSpikeRight.end())
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-32, 51.5, Math.toRadians(270)), Math.toRadians(90))
                .addTemporalMarker(0, () -> {
                    this.robot.intake.setAngle(120);
                })
                .addTemporalMarker(1.1, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(2.35, () -> {
                    robot.smartClawOpen();
                })
                .addTemporalMarker(3, () -> {
                    robot.slides.runToPosition(0);
                })
                .waitSeconds(1)
                .strafeRight(28)
                .back(5)
                .waitSeconds(3)
                .build();


        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
//        telemetry.addData("Camera State", visionPortal.getCameraState());
//        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
//            telemetry.addData("Camera State", visionPortal.getCameraState());
            telemetry.addData("Currently Recorded Position", colourMassDetectionProcessor.getRecordedPropPosition());
            telemetry.addData("Camera State", visionPortal.getCameraState());
            telemetry.addData("Currently Detected Mass Center", "x: " + colourMassDetectionProcessor.getLargestContourX() + ", y: " + colourMassDetectionProcessor.getLargestContourY());
            telemetry.addData("Currently Detected Mass Area", colourMassDetectionProcessor.getLargestContourArea());

            telemetry.update();
        }


        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */



        waitForStart();

        if (isStopRequested()) return;

        // shuts down the camera once the match starts, we dont need to look any more
        colourMassDetectionProcessor.close();
        visionPortal.close();
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }


        // gets the recorded prop position
        YoinkP2Pipeline.PropPositions recordedPropPosition = colourMassDetectionProcessor.getRecordedPropPosition();

        // now we can use recordedPropPosition to determine where the prop is! if we never saw a prop, your recorded position will be UNFOUND.
        // if it is UNFOUND, you can manually set it to any of the other positions to guess
        if (recordedPropPosition == YoinkP2Pipeline.PropPositions.UNFOUND) {
            recordedPropPosition = YoinkP2Pipeline.PropPositions.CENTER;
        }

        robot.launchSubsystemThread(telemetry);
        switch (recordedPropPosition) {
            case CENTER:
                drive.followTrajectorySequence(preloadSpikeCenter);
                drive.followTrajectorySequence(preloadBackdropCenter);
                drive.followTrajectorySequence(centerCycle1);
                drive.followTrajectorySequence(centerCycle2);
                break;
            case LEFT:
                drive.followTrajectorySequence(preloadSpikeRight);
                drive.followTrajectorySequence(preloadBackdropRight);
                drive.followTrajectorySequence(rightCycle1);
                break;
            case RIGHT:
                drive.followTrajectorySequence(preloadSpikeLeft);
                drive.followTrajectorySequence(preloadBackdropLeft);
                drive.followTrajectorySequence(leftCycle1);
                break;
        }

//        drive.followTrajectorySequence(park);



        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = drive.getPoseEstimate();

        robot.destroyThreads(telemetry);
        visionPortal.close();

        while (!isStopRequested() && opModeIsActive()) ;
    }

    public static double rad(double degrees) {
        return Math.toRadians(degrees);
    }

    public static double in(double centimeters) {
        return centimeters * 0.3837008;
    }
}