package org.firstinspires.ftc.teamcode.auton;

import org.firstinspires.ftc.teamcode.lib.Levels;

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

public class BlueBackdropV1Cancelable extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap, AllianceColor.BLUE);
        robot = new Robot(hardwareMap, true);
        robot.cv.initProp(AllianceColor.BLUE);
        Pose2d startPose = new Pose2d(-62, 11, Math.toRadians(180));
        robot.initPos();

        drive.setPoseEstimate(startPose);

        // PRELOAD PATHS
        TrajectorySequence preloadSpikeRight = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-39, 9, Math.toRadians(110)), Math.toRadians(-50))
                .build();

        TrajectorySequence preloadBackdropRight = drive.trajectorySequenceBuilder(preloadSpikeRight.end())
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-30, 49.7, Math.toRadians(270)), Math.toRadians(90))
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
                .splineTo(new Vector2d(-39.5, 11), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-49, 14, Math.toRadians(180)), Math.toRadians(180))
                .build();

        TrajectorySequence preloadBackdropCenter = drive.trajectorySequenceBuilder(preloadSpikeCenter.end())
                .setReversed(true)
                .splineTo(new Vector2d(-33, 49.7), Math.toRadians(90))
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

        TrajectorySequence preloadSpikeLeft = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(-38, 21), Math.toRadians(0))
                .forward(15)
                .build();

        TrajectorySequence preloadBackdropLeft = drive.trajectorySequenceBuilder(preloadSpikeRight.end())
                .splineToLinearHeading(new Pose2d(-39.5, 49.7, Math.toRadians(270)), Math.toRadians(90))
                .addTemporalMarker(0, () -> {
                    this.robot.intake.setAngle(120);
                })
                .addTemporalMarker(0.8, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(1.7, () -> {
                    robot.smartClawOpen();
                })

                .waitSeconds(1)
                .build();

        TrajectorySequence centerCycle1ToStack = drive.trajectorySequenceBuilder(preloadBackdropCenter.end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-9, 20), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-13, -55.1), Math.toRadians(-90))
//                .addTemporalMarker(2, () -> {
//                    robot.autoIntake(3, 170);
//                })
                .setReversed(true)
                .addTemporalMarker(2.5, () -> {
                    robot.startIntake();
                })
                .strafeRight(4)
                .forward(2.5)
                .strafeLeft(8)
                .waitSeconds(0.5)
                .build();

        TrajectorySequence centerCycle1ToBackdrop = drive.trajectorySequenceBuilder(centerCycle1ToStack.end())
                .waitSeconds(0.5)
                .addTemporalMarker(0.1, () -> {
                    robot.intake.runToPreset(Levels.INTERMEDIATE);
                })
                .addTemporalMarker(0.2, () -> {
                    robot.intake.reverseIntake();
                })
                .addTemporalMarker(0.3, () -> {
                    robot.stopIntake();
                })
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-9, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-29.5, 48.3), Math.toRadians(90))
                .addTemporalMarker(1.2, () -> {
                    robot.startIntake();
                    robot.claw.setClawOpen();
                })
                .addTemporalMarker(2.7, () -> {
                    robot.stopIntake();
                })
                .addTemporalMarker(8.5 - 4.72, () -> {
                    robot.autoCycleDepositPreset();
                })
                .addTemporalMarker(10.6 - 4.72, () -> {
                    robot.smartClawOpen();
                })
                .waitSeconds(2.5)
                .build();
//        TrajectorySequence centerCycle1ToBackgroundAuto = drive.trajectorySequenceBuilder(centerCycle1ToStack.end())
//                .waitSeconds(0.5)
//                .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(28, 48), Math.toRadians(90))
//                .addTemporalMarker(8.5, ()-> {
//                    robot.autoCycleDepositPreset();
//                })
//                .addTemporalMarker(10, ()-> {
//                    robot.smartClawOpen();
//                })
//                .waitSeconds(1.5)
//                .build();

        TrajectorySequence cycle2ToStack = drive.trajectorySequenceBuilder(centerCycle1ToBackdrop.end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-9, 20), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-14, -57), Math.toRadians(-90))
//                .addTemporalMarker(2, () -> {
//                    robot.autoIntake(3, 170);
//                })
                .setReversed(true)
                .addTemporalMarker(2.5, () -> {
                    robot.startIntake();
                })
                .strafeRight(5)
                .forward(2)
                .strafeLeft(7)

                .waitSeconds(0.5)
                .build();
        TrajectorySequence cycle2ToBackdrop = drive.trajectorySequenceBuilder(cycle2ToStack.end())
                .setReversed(true)
                .addTemporalMarker(0.1, () -> {
                    robot.intake.setAngle(90);
                })
                .addTemporalMarker(0.2, () -> {
                    robot.intake.reverseIntake();
                })
                .back(2)
                .addTemporalMarker(0.3, () -> {
                    robot.stopIntake();
                })
                .waitSeconds(0.5)
                .splineToConstantHeading(new Vector2d(-9, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-31, 48.45), Math.toRadians(90))
                .addTemporalMarker(1.2, () -> {
                    robot.startIntake();
                    robot.claw.setClawOpen();
                })
                .addTemporalMarker(2.2, () -> {
                    robot.stopIntake();
                })
                .addTemporalMarker(8.5 - 5.09, () -> {
                    robot.autoCycleDepositPreset();
                    robot.slides.runToPosition(350);
                })
                .addTemporalMarker(10.6 - 5.09, () -> {
                    robot.smartClawOpen();
                })
                .waitSeconds(2.5)
                .build();

        TrajectorySequence leftCycle1ToStack = drive.trajectorySequenceBuilder(preloadBackdropLeft.end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-9, 20), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-13, -55.1), Math.toRadians(-90))
//                .addTemporalMarker(2, () -> {
//                    robot.autoIntake(3, 170);
//                })
                .setReversed(true)
                .addTemporalMarker(2.5, () -> {
                    robot.startIntake();
                })
                .strafeRight(4)
                .forward(2.5)
                .strafeLeft(8)
                .waitSeconds(0.5)
                .build();

        TrajectorySequence leftCycle1ToBackdrop = drive.trajectorySequenceBuilder(leftCycle1ToStack.end())
                .waitSeconds(0.5)
                .addTemporalMarker(0.1, () -> {
                    robot.intake.runToPreset(Levels.INTERMEDIATE);
                })
                .addTemporalMarker(0.2, () -> {
                    robot.intake.reverseIntake();
                })
                .addTemporalMarker(0.3, () -> {
                    robot.stopIntake();
                })
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-9, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-29.5, 48.3), Math.toRadians(90))
                .addTemporalMarker(1.2, () -> {
                    robot.startIntake();
                    robot.claw.setClawOpen();
                })
                .addTemporalMarker(2.2, () -> {
                    robot.stopIntake();
                })
                .addTemporalMarker(8.5 - 4.72, () -> {
                    robot.autoCycleDepositPreset();
                })
                .addTemporalMarker(10.6 - 4.72, () -> {
                    robot.smartClawOpen();
                })
                .waitSeconds(2.5)
                .build();

        TrajectorySequence rightCycle1ToStack = drive.trajectorySequenceBuilder(preloadBackdropRight.end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-9, 20), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-13, -55.1), Math.toRadians(-90))
//                .addTemporalMarker(2, () -> {
//                    robot.autoIntake(3, 170);
//                })
                .setReversed(true)
                .addTemporalMarker(2.5, () -> {
                    robot.startIntake();
                })
                .strafeRight(4)
                .forward(2.5)
                .strafeLeft(8)
                .waitSeconds(0.5)
                .build();

        TrajectorySequence rightCycle1ToBackdrop = drive.trajectorySequenceBuilder(rightCycle1ToStack.end())
                .waitSeconds(0.5)
                .addTemporalMarker(0.1, () -> {
                    robot.intake.runToPreset(Levels.INTERMEDIATE);
                })
                .addTemporalMarker(0.2, () -> {
                    robot.intake.reverseIntake();
                })
                .addTemporalMarker(0.3, () -> {
                    robot.stopIntake();
                })
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-9, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-29.5, 48.3), Math.toRadians(90))
                .addTemporalMarker(1.2, () -> {
                    robot.startIntake();
                    robot.claw.setClawOpen();
                })
                .addTemporalMarker(2.2, () -> {
                    robot.stopIntake();
                })
                .addTemporalMarker(8.5 - 4.72, () -> {
                    robot.autoCycleDepositPreset();
                })
                .addTemporalMarker(10.6 - 4.72, () -> {
                    robot.smartClawOpen();
                })
                .waitSeconds(2.5)
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
            //telemetry.addData(drive.getAccelerationConstraint(54).toString(), " ");
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
        recordedPropPosition = YoinkP2Pipeline.PropPositions.CENTER;
        switch (recordedPropPosition) {
            case CENTER:
                drive.followTrajectorySequence(preloadSpikeCenter);
                drive.followTrajectorySequence(preloadBackdropCenter);
                drive.followTrajectorySequenceAsync(centerCycle1ToStack);
                while (drive.isBusy()) {
                    drive.update();
                    if ((robot.intakeSensor.hasPixel()[0] == true) && (robot.intakeSensor.hasPixel()[1] == true)) {//color sensors
                        telemetry.addData("trajectory", "breaking");
                        telemetry.update();
                        drive.breakFollowing();
                        break;
                    }
                }
                drive.followTrajectorySequence(centerCycle1ToBackdrop);

                drive.followTrajectorySequenceAsync(cycle2ToStack);
                while (drive.isBusy()) {
                    drive.update();
                    if ((robot.intakeSensor.hasPixel()[0] == true) && (robot.intakeSensor.hasPixel()[1] == true)) { //color sensors
                        telemetry.addData("trajectory", "breaking");
                        telemetry.update();
                        drive.breakFollowing();
                        break;
                    }
                }
                drive.followTrajectorySequence(cycle2ToBackdrop);
                break;
            case LEFT:
                drive.followTrajectorySequence(preloadSpikeLeft);
                drive.followTrajectorySequence(preloadBackdropLeft);
                drive.followTrajectorySequenceAsync(leftCycle1ToStack);
                while (drive.isBusy()) {
                    drive.update();
                    if ((robot.intakeSensor.hasPixel()[0] == true) && (robot.intakeSensor.hasPixel()[1] == true)) {//color sensors
                        telemetry.addData("trajectory", "breaking");
                        telemetry.update();
                        drive.breakFollowing();
                        break;
                    }
                }
                drive.followTrajectorySequence(leftCycle1ToBackdrop);

                drive.followTrajectorySequenceAsync(cycle2ToStack);
                while (drive.isBusy()) {
                    drive.update();
                    if ((robot.intakeSensor.hasPixel()[0] == true) && (robot.intakeSensor.hasPixel()[1] == true)) { //color sensors
                        telemetry.addData("trajectory", "breaking");
                        telemetry.update();
                        drive.breakFollowing();
                        break;
                    }
                }
                drive.followTrajectorySequence(cycle2ToBackdrop);
                break;
            case RIGHT:
                drive.followTrajectorySequence(preloadSpikeRight);
                drive.followTrajectorySequence(preloadBackdropRight);
                drive.followTrajectorySequenceAsync(rightCycle1ToStack);
                while (drive.isBusy()) {
                    drive.update();
                    if ((robot.intakeSensor.hasPixel()[0] == true) && (robot.intakeSensor.hasPixel()[1] == true)) {//color sensors
                        telemetry.addData("trajectory", "breaking");
                        telemetry.update();
                        drive.breakFollowing();
                        break;
                    }
                }
                drive.followTrajectorySequence(rightCycle1ToBackdrop);

                drive.followTrajectorySequenceAsync(cycle2ToStack);
                while (drive.isBusy()) {
                    drive.update();
                    if ((robot.intakeSensor.hasPixel()[0] == true) && (robot.intakeSensor.hasPixel()[1] == true)) { //color sensors
                        telemetry.addData("trajectory", "breaking");
                        telemetry.update();
                        drive.breakFollowing();
                        break;
                    }
                }
                drive.followTrajectorySequence(cycle2ToBackdrop);
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