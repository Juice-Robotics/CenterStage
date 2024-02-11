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

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        robot = new Robot(hardwareMap, true);
        robot.cv.initProp(AllianceColor.RED);
        Pose2d startPose = new Pose2d(62, 13, Math.toRadians(0));
        robot.initPos();

        drive.setPoseEstimate(startPose);

        // PRELOAD PATHS
        TrajectorySequence preloadSpikeLeft = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(38, 11.7, Math.toRadians(55)), Math.toRadians(30))
                .build();

        TrajectorySequence preloadBackdropLeft = drive.trajectorySequenceBuilder(preloadSpikeLeft.end())
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(30, 49, Math.toRadians(270)), Math.toRadians(90))
                .addTemporalMarker(0, () -> {
                    this.robot.intake.setAngle(120);
                })
                .addTemporalMarker(1.5, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(2.3, () -> {
                    robot.smartClawOpen();
                })
                .waitSeconds(0.8)
                .build();

        TrajectorySequence preloadSpikeCenter = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(39, 13), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(48, 13, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence preloadBackdropCenter = drive.trajectorySequenceBuilder(preloadSpikeCenter.end())
                .setReversed(true)
                .splineTo(new Vector2d(32.7, 49.5), Math.toRadians(90))
                .addTemporalMarker(0, () -> {
                    this.robot.intake.setAngle(120);
                })
                .addTemporalMarker(1.1, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(2, () -> {
                    robot.smartClawOpen();
                })
                .waitSeconds(0.2)
                .build();

        TrajectorySequence preloadSpikeRight = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(38, 25), Math.toRadians(180))
                .forward(15)
                .turn(Math.toRadians(-90))
                .build();

        TrajectorySequence preloadBackdropRight = drive.trajectorySequenceBuilder(preloadSpikeRight.end())
                .splineToLinearHeading(new Pose2d(42, 49, Math.toRadians(270)), Math.toRadians(90))
                .addTemporalMarker(0, () -> {
                    robot.intake.setAngle(120);
                })
                .addTemporalMarker(2, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(1.7, () -> {
                    robot.smartClawOpen();
                })
                .waitSeconds(3)
                .build();

        TrajectorySequence centerCycle1 = drive.trajectorySequenceBuilder(preloadBackdropCenter.end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(14, -56), Math.toRadians(-90))
//                .addTemporalMarker(2, () -> {
//                    robot.autoIntake(3, 170);
//                })
                .setReversed(true)
                .addTemporalMarker(2.9, () -> {
                    robot.intake.setAngle(182);
                    robot.intake.intakeMotor.setSpeed((float)0.3);
                })
                .back(7)
                .forward(7)
                .addTemporalMarker(4.5, () -> {
                    robot.startIntake();
                })
//                .addTemporalMarker(4.5, () -> {
//                    robot.intake.reverseIntake();
//                })
                .addTemporalMarker(5.5, () -> {
                    robot.stopIntake();
                })
                .waitSeconds(2)
                .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(30, 37.7), Math.toRadians(90))
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(30, 47.5), Math.toRadians(90))
                .addTemporalMarker(7, () -> {
                    robot.startIntake();
                    robot.claw.setClawOpen();
                })
                .addTemporalMarker(8, () -> {
                    robot.stopIntake();
                })
                .addTemporalMarker(10, () -> {
                    Pose2d newPose = robot.cv.relocalizeUsingBackdrop(drive.getPoseEstimate());
                    drive.setPoseEstimate(newPose);
                })
                .addTemporalMarker(10, ()-> {
                    robot.slides.runToPosition(50);
                    robot.autoCycleDepositPreset();
                })
                .addTemporalMarker(12.7, ()-> {
                    robot.smartClawOpen();
                })
                .waitSeconds(2)
                .build();

        TrajectorySequence centerCycle2 = drive.trajectorySequenceBuilder(centerCycle1.end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(16, -57.6), Math.toRadians(-90))
//                .addTemporalMarker(2, () -> {
//                    robot.autoIntake(3, 170);
//                })
                .setReversed(true)
//                .addTemporalMarker(2.3, () -> {
//                    robot.intake.setAngle(160);
//                })
                .addTemporalMarker(3, () -> {
                    robot.startIntake();
                })
//                .addTemporalMarker(4.5, () -> {
//                    robot.intake.reverseIntake();
//                })
                .addTemporalMarker(4.5, () -> {
                    robot.stopIntake();
                })
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(30, 47.4), Math.toRadians(90))
                .addTemporalMarker(5.5, () -> {
                    robot.startAutoIntake();
                    robot.claw.setClawOpen();
                })
                .addTemporalMarker(6, () -> {
                    robot.stopIntake();
                })
                .addTemporalMarker(6.8, ()-> {
                    robot.slides.runToPosition(50);
                    robot.autoCycleDepositPreset();
                })
                .addTemporalMarker(8.5, ()-> {
                    robot.smartClawOpen();
                })
                .waitSeconds(2)
                .build();

        TrajectorySequence leftCycle1 = drive.trajectorySequenceBuilder(preloadBackdropLeft.end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(16, -55.8), Math.toRadians(-90))
//                .addTemporalMarker(2, () -> {
//                    robot.autoIntake(3, 170);
//                })
                .setReversed(true)
                .addTemporalMarker(2.3, () -> {
                    robot.intake.setAngle(160);
                })
                .back(8)
                .forward(8)
                .addTemporalMarker(4.5, () -> {
                    robot.startIntake();
                })
//                .addTemporalMarker(4.5, () -> {
//                    robot.intake.reverseIntake();
//                })
                .addTemporalMarker(5.5, () -> {
                    robot.stopIntake();
                })
                .waitSeconds(0.2)
                .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(30, 37.7), Math.toRadians(90))
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(30, 47.7), Math.toRadians(90))
//                .addTemporalMarker(6, () -> {
//                    robot.startAutoIntake();
//                    robot.claw.setClawOpen();
//                })
                .addTemporalMarker(6.5, () -> {
                    robot.stopIntake();
                })
                .addTemporalMarker(8.2, ()-> {
                    robot.autoCycleDepositPreset();
                })
                .addTemporalMarker(9, ()-> {
                    robot.smartClawOpen();
                })
                .waitSeconds(1.2)
                .build();

        TrajectorySequence leftCycle2 = drive.trajectorySequenceBuilder(leftCycle1.end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(16, -55.8), Math.toRadians(-90))
//                .addTemporalMarker(2, () -> {
//                    robot.autoIntake(3, 170);
//                })
                .setReversed(true)
                .addTemporalMarker(2.3, () -> {
                    robot.intake.setAngle(160);
                })
                .back(8)
                .forward(8)
                .addTemporalMarker(4.5, () -> {
                    robot.startIntake();
                })
//                .addTemporalMarker(4.5, () -> {
//                    robot.intake.reverseIntake();
//                })
                .addTemporalMarker(5.5, () -> {
                    robot.stopIntake();
                })
                .waitSeconds(0.2)
                .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(30, 47.7), Math.toRadians(90))
//                .addTemporalMarker(6, () -> {
//                    robot.startAutoIntake();
//                    robot.claw.setClawOpen();
//                })
                .addTemporalMarker(6.5, () -> {
                    robot.stopIntake();
                })
                .addTemporalMarker(7, () -> {
                    Pose2d newPose = robot.cv.relocalizeUsingBackdrop(drive.getPoseEstimate());
                    drive.setPoseEstimate(newPose);
                })
                .addTemporalMarker(7.2, ()-> {
                    robot.autoCycleDepositPreset();
                })
                .addTemporalMarker(8, ()-> {
                    robot.smartClawOpen();
                })
                .waitSeconds(1.2)
                .build();

        TrajectorySequence rightCycle1 = drive.trajectorySequenceBuilder(preloadBackdropRight.end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(16, -55.8), Math.toRadians(-90))
//                .addTemporalMarker(2, () -> {
//                    robot.autoIntake(3, 170);
//                })
                .setReversed(true)
                .addTemporalMarker(2.3, () -> {
                    robot.intake.setAngle(160);
                })
                .back(8)
                .forward(8)
                .addTemporalMarker(4.5, () -> {
                    robot.startIntake();
                })
//                .addTemporalMarker(4.5, () -> {
//                    robot.intake.reverseIntake();
//                })
                .addTemporalMarker(5.5, () -> {
                    robot.stopIntake();
                })
                .waitSeconds(0.2)
                .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(30, 47.7), Math.toRadians(90))
//                .addTemporalMarker(6, () -> {
//                    robot.startAutoIntake();
//                    robot.claw.setClawOpen();
//                })
                .addTemporalMarker(6.5, () -> {
                    robot.stopIntake();
                })
                .addTemporalMarker(7.2, ()-> {
                    robot.autoCycleDepositPreset();
                })
                .addTemporalMarker(8, ()-> {
                    robot.smartClawOpen();
                })
                .waitSeconds(1.2)
                .build();

        TrajectorySequence rightCycle2 = drive.trajectorySequenceBuilder(rightCycle1.end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(16, -55.8), Math.toRadians(-90))
//                .addTemporalMarker(2, () -> {
//                    robot.autoIntake(3, 170);
//                })
                .setReversed(true)
                .addTemporalMarker(2.3, () -> {
                    robot.intake.setAngle(160);
                })
                .back(8)
                .forward(8)
                .addTemporalMarker(4.5, () -> {
                    robot.startIntake();
                })
//                .addTemporalMarker(4.5, () -> {
//                    robot.intake.reverseIntake();
//                })
                .addTemporalMarker(5.5, () -> {
                    robot.stopIntake();
                })
                .waitSeconds(0.2)
                .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(30, 47.7), Math.toRadians(90))
//                .addTemporalMarker(6, () -> {
//                    robot.startAutoIntake();
//                    robot.claw.setClawOpen();
//                })
                .addTemporalMarker(6.5, () -> {
                    robot.stopIntake();
                })
                .addTemporalMarker(7.2, ()-> {
                    robot.autoCycleDepositPreset();
                })
                .addTemporalMarker(8, ()-> {
                    robot.smartClawOpen();
                })
                .waitSeconds(1.2)
                .build();

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
//        telemetry.addData("Camera State", visionPortal.getCameraState());
//        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
//            telemetry.addData("Camera State", visionPortal.getCameraState());
            telemetry.addData("Currently Recorded Position", robot.cv.colourMassDetectionProcessor.getRecordedPropPosition());
            telemetry.addData("Camera State", robot.cv.visionPortal.getCameraState());
            telemetry.addData("Currently Detected Mass Center", "x: " + robot.cv.colourMassDetectionProcessor.getLargestContourX() + ", y: " + robot.cv.colourMassDetectionProcessor.getLargestContourY());
            telemetry.addData("Currently Detected Mass Area", robot.cv.colourMassDetectionProcessor.getLargestContourArea());

            telemetry.update();
        }


        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */



        waitForStart();

        if (isStopRequested()) return;

        // gets the recorded prop position
        YoinkP2Pipeline.PropPositions recordedPropPosition = robot.cv.colourMassDetectionProcessor.getRecordedPropPosition();

        // now we can use recordedPropPosition to determine where the prop is! if we never saw a prop, your recorded position will be UNFOUND.
        // if it is UNFOUND, you can manually set it to any of the other positions to guess
        if (recordedPropPosition == YoinkP2Pipeline.PropPositions.UNFOUND) {
            recordedPropPosition = YoinkP2Pipeline.PropPositions.CENTER;
        }

        // shuts down the camera once the match starts, we dont need to look any more
        robot.cv.switchToAprilTags();

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
                break;
        }

//        drive.followTrajectorySequence(park);



        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = drive.getPoseEstimate();

        robot.destroyThreads(telemetry);
        robot.cv.kill();

        while (!isStopRequested() && opModeIsActive()) ;
    }

    public static double rad(double degrees) {
        return Math.toRadians(degrees);
    }

    public static double in(double centimeters) {
        return centimeters * 0.3837008;
    }
}