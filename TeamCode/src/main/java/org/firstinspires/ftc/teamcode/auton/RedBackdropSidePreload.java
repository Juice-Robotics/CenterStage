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
import org.firstinspires.ftc.teamcode.lib.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.vision.YoinkP2Pipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;

@Config
@Autonomous(group = "drive")

public class RedBackdropSidePreload extends LinearOpMode {
    Robot robot;
    private VisionPortal visionPortal;
    private YoinkP2Pipeline colourMassDetectionProcessor;
    AprilTagProcessor processor;

    @Override
    public void runOpMode() throws InterruptedException {
        Scalar lower = new Scalar(125, 60, 50); // the lower hsv threshold for your detection
        Scalar upper = new Scalar(190, 255, 255); // the upper hsv threshold for your detection
        double minArea = 5000; // the minimum area for the detection to consider for your prop

        colourMassDetectionProcessor = new YoinkP2Pipeline(
                lower,
                upper,
                () -> minArea, // these are lambda methods, in case we want to change them while the match is running, for us to tune them or something
                () -> 213, // the left dividing line, in this case the left third of the frame
                () -> 606 // the left dividing line, in this case the right third of the frame
        );
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
                .addProcessor(colourMassDetectionProcessor)
                .build();

        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        robot = new Robot(hardwareMap, true);
        Pose2d startPose = new Pose2d(62, 13, Math.toRadians(0));
        robot.initPos();

        drive.setPoseEstimate(startPose);

        // PRELOAD PATHS
        TrajectorySequence preloadSpikeLeft = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(40, 7, Math.toRadians(55)), Math.toRadians(30))
                .build();

        TrajectorySequence preloadBackdropLeft = drive.trajectorySequenceBuilder(preloadSpikeLeft.end())
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(30, 48, Math.toRadians(270)), Math.toRadians(90))
                .addTemporalMarker(0, () -> {
                    this.robot.intake.setAngle(120);
                })
                .addTemporalMarker(2, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(3.5, () -> {
                    robot.smartClawOpen();
                })
                .waitSeconds(3)
                .build();

        TrajectorySequence preloadSpikeCenter = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(40, 13), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(48, 13, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence preloadBackdropCenter = drive.trajectorySequenceBuilder(preloadSpikeCenter.end())
                .setReversed(true)
                .splineTo(new Vector2d(33.3, 50), Math.toRadians(90))
                .addTemporalMarker(0, () -> {
                    this.robot.intake.setAngle(120);
                })
                .addTemporalMarker(1.1, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(1.95, () -> {
                    robot.smartClawOpen();
                })
                .waitSeconds(0.2)
                .build();

        TrajectorySequence preloadSpikeRight = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(38, 26), Math.toRadians(180))
                .forward(15)
                .build();

        TrajectorySequence preloadBackdropRight = drive.trajectorySequenceBuilder(preloadSpikeRight.end())
                .splineToLinearHeading(new Pose2d(42, 50, Math.toRadians(270)), Math.toRadians(90))
                .addTemporalMarker(0, () -> {
                    this.robot.intake.setAngle(120);
                })
                .addTemporalMarker(0.8, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(1.6, () -> {
                    robot.smartClawOpen();
                })
                .waitSeconds(0.2)
                .build();

        TrajectorySequence centerCycle1 = drive.trajectorySequenceBuilder(preloadBackdropCenter.end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(13, -55.4), Math.toRadians(-90))
//                .addTemporalMarker(2, () -> {
//                    robot.autoIntake(3, 170);
//                })
                .setReversed(true)
                .addTemporalMarker(2.5, () -> {
                    robot.startIntake();
                })
                .strafeLeft(4)
                .forward(2.5)
                .strafeRight(8)
                .addTemporalMarker(5, () -> {
                    robot.intake.setAngle(90);
                })
                .addTemporalMarker(5.1, () -> {
                    robot.intake.reverseIntake();
                })
                .addTemporalMarker(5.2, () -> {
                    robot.stopIntake();
                })
                .waitSeconds(0.5)
                .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(30, 47.44), Math.toRadians(90))
                .addTemporalMarker(6, () -> {
                    robot.startAutoIntake();
                })
                .addTemporalMarker(6.8, () -> {
                    robot.stopIntake();
                })
                .addTemporalMarker(8.2, ()-> {
                    robot.autoCycleDepositPreset();
                })
                .addTemporalMarker(9.5, ()-> {
                    robot.smartClawOpen();
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence centerCycle2 = drive.trajectorySequenceBuilder(centerCycle1.end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(7, 20), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(12, -57), Math.toRadians(-90))
//                .addTemporalMarker(2, () -> {
//                    robot.autoIntake(3, 170);
//                })
                .setReversed(true)
                .addTemporalMarker(2.5, () -> {
                    robot.startIntake();
                })
                .strafeLeft(5)
                .forward(2)
                .strafeRight(7)
                .addTemporalMarker(4.5, () -> {
                    robot.intake.setAngle(90);
                })
                .addTemporalMarker(4.6, () -> {
                    robot.intake.reverseIntake();
                })
                .back(2)
                .addTemporalMarker(4.7, () -> {
                    robot.stopIntake();
                })
                .waitSeconds(0.5)
                .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(30, 47.44), Math.toRadians(90))
                .addTemporalMarker(6, () -> {
                    robot.startAutoIntake();
                })
                .addTemporalMarker(6.8, () -> {
                    robot.stopIntake();
                })
                .addTemporalMarker(8.5, ()-> {
                    robot.autoCycleDepositPreset();
                })
                .addTemporalMarker(10.5, ()-> {
                    robot.smartClawOpen();
                })
                .addTemporalMarker(13, ()-> {
                    robot.slides.runToPosition(0);
                })
                .waitSeconds(2)
                .strafeRight(21)
                .back(7)
                .waitSeconds(2)
                .build();

        TrajectorySequence leftCycle1 = drive.trajectorySequenceBuilder(preloadBackdropLeft.end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(13, -55.4), Math.toRadians(-90))
//                .addTemporalMarker(2, () -> {
//                    robot.autoIntake(3, 170);
//                })
                .setReversed(true)
                .addTemporalMarker(2.5, () -> {
                    robot.startIntake();
                })
                .strafeLeft(4)
                .forward(2.5)
                .strafeRight(8)
                .addTemporalMarker(5, () -> {
                    robot.intake.setAngle(90);
                })
                .addTemporalMarker(5.1, () -> {
                    robot.intake.reverseIntake();
                })
                .addTemporalMarker(5.2, () -> {
                    robot.stopIntake();
                })
                .waitSeconds(0.5)
                .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(30, 47.44), Math.toRadians(90))
                .addTemporalMarker(6, () -> {
                    robot.startAutoIntake();
                })
                .addTemporalMarker(6.8, () -> {
                    robot.stopIntake();
                })
                .addTemporalMarker(8.2, ()-> {
                    robot.autoCycleDepositPreset();
                })
                .addTemporalMarker(9.5, ()-> {
                    robot.smartClawOpen();
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence leftCycle2 = drive.trajectorySequenceBuilder(leftCycle1.end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(7, 20), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(12, -57), Math.toRadians(-90))
//                .addTemporalMarker(2, () -> {
//                    robot.autoIntake(3, 170);
//                })
                .setReversed(true)
                .addTemporalMarker(2.5, () -> {
                    robot.startIntake();
                })
                .strafeLeft(5)
                .forward(2)
                .strafeRight(7)
                .addTemporalMarker(4.5, () -> {
                    robot.intake.setAngle(90);
                })
                .addTemporalMarker(4.6, () -> {
                    robot.intake.reverseIntake();
                })
                .back(2)
                .addTemporalMarker(4.7, () -> {
                    robot.stopIntake();
                })
                .waitSeconds(0.5)
                .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(30, 47.44), Math.toRadians(90))
                .addTemporalMarker(6, () -> {
                    robot.startAutoIntake();
                })
                .addTemporalMarker(6.8, () -> {
                    robot.stopIntake();
                })
                .addTemporalMarker(8.5, ()-> {
                    robot.autoCycleDepositPreset();
                })
                .addTemporalMarker(10.5, ()-> {
                    robot.smartClawOpen();
                })
                .addTemporalMarker(13, ()-> {
                    robot.slides.runToPosition(0);
                })
                .waitSeconds(2)
                .strafeRight(21)
                .back(7)
                .waitSeconds(2)
                .build();

        TrajectorySequence rightCycle1 = drive.trajectorySequenceBuilder(preloadBackdropRight.end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(13, -54.9), Math.toRadians(-90))
//                .addTemporalMarker(2, () -> {
//                    robot.autoIntake(3, 170);
//                })
                .setReversed(true)
                .addTemporalMarker(2.5, () -> {
                    robot.startIntake();
                })
                .strafeLeft(4)
                .forward(2.5)
                .strafeRight(8)
                .addTemporalMarker(5, () -> {
                    robot.intake.setAngle(90);
                })
                .addTemporalMarker(5.1, () -> {
                    robot.intake.reverseIntake();
                })
                .addTemporalMarker(5.2, () -> {
                    robot.stopIntake();
                })
                .waitSeconds(0.5)
                .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(30, 47.54), Math.toRadians(90))
                .addTemporalMarker(6, () -> {
                    robot.startAutoIntake();
                })
                .addTemporalMarker(6.8, () -> {
                    robot.stopIntake();
                })
                .addTemporalMarker(8.2, ()-> {
                    robot.autoCycleDepositPreset();
                })
                .addTemporalMarker(9.5, ()-> {
                    robot.smartClawOpen();
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence rightCycle2 = drive.trajectorySequenceBuilder(rightCycle1.end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(7, 20), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(12, -57), Math.toRadians(-90))
//                .addTemporalMarker(2, () -> {
//                    robot.autoIntake(3, 170);
//                })
                .setReversed(true)
                .addTemporalMarker(2.5, () -> {
                    robot.startIntake();
                })
                .strafeLeft(5)
                .forward(2)
                .strafeRight(7)
                .addTemporalMarker(4.5, () -> {
                    robot.intake.setAngle(90);
                })
                .addTemporalMarker(4.6, () -> {
                    robot.intake.reverseIntake();
                })
                .back(2)
                .addTemporalMarker(4.7, () -> {
                    robot.stopIntake();
                })
                .waitSeconds(0.5)
                .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(30, 47.44), Math.toRadians(90))
                .addTemporalMarker(6, () -> {
                    robot.startAutoIntake();
                })
                .addTemporalMarker(6.8, () -> {
                    robot.stopIntake();
                })
                .addTemporalMarker(8.5, ()-> {
                    robot.autoCycleDepositPreset();
                })
                .addTemporalMarker(10.5, ()-> {
                    robot.smartClawOpen();
                })
                .addTemporalMarker(13, ()-> {
                    robot.slides.runToPosition(0);
                })
                .waitSeconds(2)
                .strafeRight(21)
                .back(7)
                .waitSeconds(2)
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
                drive.followTrajectorySequence(preloadSpikeLeft);
                drive.followTrajectorySequence(preloadBackdropLeft);
                drive.followTrajectorySequence(leftCycle1);
                drive.followTrajectorySequence(leftCycle2);
                break;
            case RIGHT:
                drive.followTrajectorySequence(preloadSpikeRight);
                drive.followTrajectorySequence(preloadBackdropRight);
                drive.followTrajectorySequence(rightCycle1);
                drive.followTrajectorySequence(rightCycle2);
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