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
import org.firstinspires.ftc.teamcode.subsystems.vision.TeamElementCVProcessor;
import org.firstinspires.ftc.teamcode.subsystems.vision.YoinkElementCVProcessor;
import org.firstinspires.ftc.teamcode.subsystems.vision.YoinkP2Pipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;

import java.util.List;

@Config
@Autonomous(group = "drive")

public class BlueBackdropSidePreload extends LinearOpMode {
    Robot robot;
    private VisionPortal visionPortal;
    private YoinkP2Pipeline colourMassDetectionProcessor;
//    AprilTagDetection aprilTagDetection;
//    List<AprilTagDetection> aprilTagDetections;
    AprilTagProcessor processor;

    @Override
    public void runOpMode() throws InterruptedException {
        processor = AprilTagProcessor.easyCreateWithDefaults();
        Scalar lower = new Scalar(90, 60, 40); // the lower hsv threshold for your detection
        Scalar upper = new Scalar(130, 255, 255); // the upper hsv threshold for your detection
        double minArea = 100; // the minimum area for the detection to consider for your prop

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
        TrajectorySequence preloadSpikeRight = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-30, 10, Math.toRadians(90)), Math.toRadians(0))
                .forward(15)
                .turn(Math.toRadians(180))
                .build();

        TrajectorySequence preloadBackdropRight = drive.trajectorySequenceBuilder(preloadSpikeRight.end())
                .splineToSplineHeading(new Pose2d(-30, 30, Math.toRadians(90)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-30, 50, Math.toRadians(270)), Math.toRadians(90))
                .addTemporalMarker(0, () -> {
                    this.robot.intake.setAngle(120);
                })
                .addTemporalMarker(2, () -> {
                    //robot.relocalization.relocalizeUsingBackdrop(robot.drive.getPoseEstimate());
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(3.5, () -> {
                    robot.smartClawOpen();
                })
                .waitSeconds(3)
//                .strafeLeft(20)
//                .back(10)
                .build();

        TrajectorySequence preloadSpikeCenter = drive.trajectorySequenceBuilder(startPose)
                .splineToSplineHeading(new Pose2d(-30, 30, Math.toRadians(90)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-30, 50, Math.toRadians(270)), Math.toRadians(90))
                .build();

        TrajectorySequence preloadBackdropCenter = drive.trajectorySequenceBuilder(preloadSpikeCenter.end())
                .setReversed(true)
                .splineTo(new Vector2d(-33, 50), Math.toRadians(90))
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
//                .strafeLeft(22)
//                .back(10)
                .build();

        TrajectorySequence preloadSpikeLeft = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(-38, 21), Math.toRadians(0))
                .forward(15)
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence preloadBackdropLeft = drive.trajectorySequenceBuilder(preloadSpikeLeft.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-42, 50, Math.toRadians(270)), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(-29, 52), Math.toRadians(90))
                .addTemporalMarker(0, () -> {
                    this.robot.intake.setAngle(120);
                })
                .addTemporalMarker(2, () -> {
                    //robot.relocalization.relocalizeUsingBackdrop(robot.drive.getPoseEstimate());
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(3.5, () -> {
                    robot.smartClawOpen();
                })
                .waitSeconds(3)
//                .strafeLeft(30)
//                .back(10)
                .build();

        TrajectorySequence centerCycle1 = drive.trajectorySequenceBuilder(preloadBackdropCenter.end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-12, -55), Math.toRadians(-90))
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
                .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-28, 48), Math.toRadians(90))
                .addTemporalMarker(7.5, () -> {
                    robot.autoCycleDepositPreset();
                })
                .addTemporalMarker(9, () -> {
                    robot.smartClawOpen();
                })
                .waitSeconds(0.5)
                .build();

        TrajectorySequence centerCycle2 = drive.trajectorySequenceBuilder(centerCycle1.end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-7, 20), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-12, -57), Math.toRadians(-90))
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
                .addTemporalMarker(5, () -> {
                    robot.intake.setAngle(90);
                })
                .addTemporalMarker(5.1, () -> {
                    robot.intake.reverseIntake();
                })
                .back(2)
                .addTemporalMarker(5.2, () -> {
                    robot.stopIntake();
                })
                .waitSeconds(0.5)
                .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-29, 47), Math.toRadians(90))
                .addTemporalMarker(7.5, () -> {
                    robot.autoCycleDepositPreset();
                })
                .addTemporalMarker(9, () -> {
                    robot.smartClawOpen();
                })
                .addTemporalMarker(9.5, () -> {
                    robot.slides.runToPosition(0);
                })
                .waitSeconds(0.5)
                .build();

        TrajectorySequence leftCycle1 = drive.trajectorySequenceBuilder(preloadBackdropLeft.end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-12, -52), Math.toRadians(-90))
//                .addTemporalMarker(2, () -> {
//                    robot.autoIntake(3, 170);
//                })
                .setReversed(true)
                .waitSeconds(4)
                .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-34, 49), Math.toRadians(90))
                .build();

        TrajectorySequence rightCycle1 = drive.trajectorySequenceBuilder(preloadBackdropRight.end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-12, -52), Math.toRadians(-90))
//                .addTemporalMarker(2, () -> {
//                    robot.autoIntake(3, 170);
//                })
                .setReversed(true)
                .waitSeconds(4)
                .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-34, 49), Math.toRadians(90))
                .build();


//        TrajectorySequence park = drive.trajectorySequenceBuilder(prel.end())
//                .strafeLeft(10)
//                .build();


        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */


//        telemetry.addData("Camera State", visionPortal.getCameraState());
//        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Currently Recorded Position", colourMassDetectionProcessor.getRecordedPropPosition());
            telemetry.addData("Camera State", visionPortal.getCameraState());
            telemetry.addData("Currently Detected Mass Center", "x: " + colourMassDetectionProcessor.getLargestContourX() + ", y: " + colourMassDetectionProcessor.getLargestContourY());
            telemetry.addData("Currently Detected Mass Area", colourMassDetectionProcessor.getLargestContourArea());

            telemetry.update();
        }
        visionPortal.close();

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        waitForStart();

        if (isStopRequested()) return;


//        // if it is UNFOUND, you can manually set it to any of the other positions to guess
        colourMassDetectionProcessor.close();
        visionPortal.close();
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopLiveView();
            visionPortal.stopStreaming();
        }


        // gets the recorded prop position
        YoinkP2Pipeline.PropPositions recordedPropPosition = colourMassDetectionProcessor.getRecordedPropPosition();

        robot.slides.launchAsThread(telemetry);
        switch (recordedPropPosition) {
            case CENTER:
                drive.followTrajectorySequence(preloadSpikeCenter);
                drive.followTrajectorySequence(preloadBackdropCenter);
                break;
            case LEFT:
                drive.followTrajectorySequence(preloadSpikeLeft);
                drive.followTrajectorySequence(preloadBackdropLeft);
                break;
            case RIGHT:
                drive.followTrajectorySequence(preloadSpikeRight);
                drive.followTrajectorySequence(preloadBackdropRight);
                break;
        }

//     drive.followTrajectorySequence(park);
        //drive.followTrajectorySequence(cycle1);



        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = drive.getPoseEstimate();

        robot.slides.destroyThreads(telemetry);


        while (!isStopRequested() && opModeIsActive()) ;
    }

    public static double rad(double degrees) {
        return Math.toRadians(degrees);
    }

    public static double in(double centimeters) {
        return centimeters * 0.3837008;
    }
}