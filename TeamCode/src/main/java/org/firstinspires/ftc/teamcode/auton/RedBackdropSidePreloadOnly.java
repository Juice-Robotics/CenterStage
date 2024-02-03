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
import org.firstinspires.ftc.teamcode.subsystems.vision.CVMaster;
import org.firstinspires.ftc.teamcode.subsystems.vision.YoinkP2Pipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Scalar;

@Config
@Autonomous(group = "drive")

public class RedBackdropSidePreloadOnly extends LinearOpMode {
    Robot robot;
    CVMaster cv;

    @Override
    public void runOpMode() throws InterruptedException {
        cv = new CVMaster(hardwareMap, AllianceColor.RED);
        cv.init();

        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        robot = new Robot(hardwareMap, true);
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
                .splineToLinearHeading(new Pose2d(31, 49, Math.toRadians(270)), Math.toRadians(90))
                .addTemporalMarker(0, () -> {
                    this.robot.intake.setAngle(120);
                })
                .addTemporalMarker(1.5, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(2.3, () -> {
                    robot.smartClawOpen();
                })
//                .addTemporalMarker(4, ()-> {
//                    robot.slides.runToPosition(0);
//                })
                .waitSeconds(2)
//                .strafeRight(22)
                .forward(10)
                .build();

        TrajectorySequence preloadSpikeCenter = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(40, 13), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(48, 13, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence preloadBackdropCenter = drive.trajectorySequenceBuilder(preloadSpikeCenter.end())
                .setReversed(true)
                .splineTo(new Vector2d(32.5, 49.5), Math.toRadians(90))
                .addTemporalMarker(0, () -> {
                    this.robot.intake.setAngle(120);
                })
                .addTemporalMarker(1.1, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(2, () -> {
                    robot.smartClawOpen();
                })
//                .addTemporalMarker(4, ()-> {
//                    robot.slides.runToPosition(0);
//                })
                .waitSeconds(2)
//                .strafeRight(25)
                .forward(19)
                .waitSeconds(2)
                .build();

        TrajectorySequence preloadSpikeRight = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(38, 26), Math.toRadians(180))
                .forward(15)
                .build();

        TrajectorySequence preloadBackdropRight = drive.trajectorySequenceBuilder(preloadSpikeRight.end())
                .splineToLinearHeading(new Pose2d(39.5, 49, Math.toRadians(270)), Math.toRadians(90))
                .addTemporalMarker(0, () -> {
                    this.robot.intake.setAngle(120);
                })
                .addTemporalMarker(0.8, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(1.7, () -> {
                    robot.smartClawOpen();
                })
//                .addTemporalMarker(4, ()-> {
//                    robot.slides.runToPosition(0);
//                })
                .waitSeconds(2)
//                .strafeRight(29)
                .forward(10)
                .build();

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
//        telemetry.addData("Camera State", visionPortal.getCameraState());
//        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
//            telemetry.addData("Camera State", visionPortal.getCameraState());
            telemetry.addData("Currently Recorded Position", cv.colourMassDetectionProcessor.getRecordedPropPosition());
            telemetry.addData("Camera State", cv.visionPortal.getCameraState());
            telemetry.addData("Currently Detected Mass Center", "x: " + cv.colourMassDetectionProcessor.getLargestContourX() + ", y: " + cv.colourMassDetectionProcessor.getLargestContourY());
            telemetry.addData("Currently Detected Mass Area", cv.colourMassDetectionProcessor.getLargestContourArea());

            telemetry.update();
        }


        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */



        waitForStart();

        if (isStopRequested()) return;


        // gets the recorded prop position
        YoinkP2Pipeline.PropPositions recordedPropPosition = cv.colourMassDetectionProcessor.getRecordedPropPosition();

        // now we can use recordedPropPosition to determine where the prop is! if we never saw a prop, your recorded position will be UNFOUND.
        // if it is UNFOUND, you can manually set it to any of the other positions to guess
        if (recordedPropPosition == YoinkP2Pipeline.PropPositions.UNFOUND) {
            recordedPropPosition = YoinkP2Pipeline.PropPositions.CENTER;
        }

        // shuts down the camera once the match starts, we dont need to look any more
        cv.switchToAprilTags();

        robot.launchSubsystemThread(telemetry);
        Pose2d newPose = new Pose2d(0,0,0);
        switch (recordedPropPosition) {
            case CENTER:
                drive.followTrajectorySequence(preloadSpikeCenter);
                drive.followTrajectorySequence(preloadBackdropCenter);
                newPose = cv.relocalizeUsingBackdrop(drive.getPoseEstimate());
                telemetry.addData("old pose", drive.getPoseEstimate());
                telemetry.addData("new pose", newPose);
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

//        drive.followTrajectorySequence(park);



        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = drive.getPoseEstimate();

        robot.destroyThreads(telemetry);
        cv.kill();

        telemetry.addData("old pose", drive.getPoseEstimate());
        telemetry.addData("new pose", newPose);
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) {
            telemetry.addData("old pose", drive.getPoseEstimate());
            telemetry.addData("new pose", newPose);
            telemetry.update();
        };
    }

    public static double rad(double degrees) {
        return Math.toRadians(degrees);
    }

    public static double in(double centimeters) {
        return centimeters * 0.3837008;
    }
}