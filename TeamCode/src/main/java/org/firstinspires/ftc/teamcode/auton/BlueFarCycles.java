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

public class BlueFarCycles extends LinearOpMode {
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
        Pose2d startPose = new Pose2d(-62, -34, Math.toRadians(180));
        robot.initPos();

        drive.setPoseEstimate(startPose);

        // PRELOAD PATHS
        TrajectorySequence preloadSpikeLeft = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-32, -34, Math.toRadians(90)), Math.toRadians(90))
                .addTemporalMarker(1.8, () -> {
                    //release pixel from intake
                })
                .waitSeconds(1)
                //stack
                .splineToLinearHeading(new Pose2d(-12, -57, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(1.5)
                .addTemporalMarker(0, () -> {
                    this.robot.intake.setAngle(120);
                })
                .addTemporalMarker(0.7, () -> {
                    //intake
                })
                .addTemporalMarker(2.1, () -> {
                    robot.stopIntake();
                })
                .build();

        TrajectorySequence preloadBackdropLeft = drive.trajectorySequenceBuilder(preloadSpikeLeft.end())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-60, -15, Math.toRadians(-90)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-42, 51, Math.toRadians(-90)), Math.toRadians(90))
                .addTemporalMarker(0, () -> {
                    this.robot.intake.setAngle(120);
                })
                .addTemporalMarker(2, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(3, () -> {
                    robot.smartClawOpen();
                })
                .addTemporalMarker(3.4, () -> {
                    robot.slides.runToPosition(0);
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence preloadSpikeCenter = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .back(48)
                .waitSeconds(1)
                //.splineToLinearHeading(new Pose2d(-48, 13, Math.toRadians(180)), Math.toRadians(0))
                .addTemporalMarker(1.8, () -> {
                    //release pixel from intake
                })
                //stack
                .strafeLeft(8)
                .splineToSplineHeading(new Pose2d(-12, -57, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(1.5)
                .build();

        TrajectorySequence preloadBackdropCenter = drive.trajectorySequenceBuilder(preloadSpikeCenter.end())
                //break
                //.setReversed(true)
                .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-35, 47.7), Math.toRadians(90))
                .addTemporalMarker(0, () -> {
                    this.robot.intake.setAngle(120);
                })
                .addTemporalMarker(1.1, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(2.1, () -> {
                    robot.smartClawOpen();
                })
                .addTemporalMarker(2.5, () -> {
                    robot.slides.runToPosition(0);
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence preloadSpikeRight = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-22, -47, Math.toRadians(180)), Math.toRadians(180))
                .waitSeconds(1)

                .addTemporalMarker(1.8, () -> {
                    //release pixel from intake
                })
                //stack
                .splineToLinearHeading(new Pose2d(-12, -57, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(1.5)
                .addTemporalMarker(3.5, () -> {
                    //this.robot.intake.setAngle(120);
                })
                .addTemporalMarker(3.7, () -> {
                    //intake
                })
                .addTemporalMarker(5.2, () -> {
                    //robot.stopIntake();
                })
                .build();

        TrajectorySequence preloadBackdropRight = drive.trajectorySequenceBuilder(preloadSpikeRight.end())
                .splineToConstantHeading(new Vector2d(-10, 25), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-42, 51), Math.toRadians(90))
                .addTemporalMarker(0, () -> {
                    this.robot.intake.setAngle(120);
                })
                .addTemporalMarker(1.1, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(2.1, () -> {
                    robot.smartClawOpen();
                })
                .addTemporalMarker(2.5, () -> {
                    robot.slides.runToPosition(0);
                })
                .waitSeconds(1)
                .build();
        TrajectorySequence leftCycle1 = drive.trajectorySequenceBuilder(preloadBackdropLeft.end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-13, -55.8), Math.toRadians(-90))
//                .addTemporalMarker(2, () -> {
//                    robot.autoIntake(3, 170);
//                })
                .setReversed(true)
                .addTemporalMarker(3.2, () -> {
                    robot.startIntake();
                })
                .addTemporalMarker(4.7, () -> {
                    robot.stopIntake();
                })
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-30, 47.7), Math.toRadians(90))

                .addTemporalMarker(5, () -> {
                    robot.startAutoIntake();
                    robot.claw.setClawOpen();
                })
                .addTemporalMarker(6, () -> {
                    robot.stopIntake();
                })
                .addTemporalMarker(7.3, ()-> {
                    robot.autoCycleDepositPreset();
                })
                .addTemporalMarker(8, ()-> {
                    robot.smartClawOpen();
                })
                .waitSeconds(1.2)
                .build();
        TrajectorySequence centerCycle1 = drive.trajectorySequenceBuilder(preloadBackdropCenter.end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-13, -55.8), Math.toRadians(-90))
//                .addTemporalMarker(2, () -> {
//                    robot.autoIntake(3, 170);
//                })
                .setReversed(true)
                .addTemporalMarker(3.2, () -> {
                    robot.startIntake();
                })
                .addTemporalMarker(4.7, () -> {
                    robot.stopIntake();
                })
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-30, 47.7), Math.toRadians(90))

                .addTemporalMarker(5, () -> {
                    robot.startAutoIntake();
                    robot.claw.setClawOpen();
                })
                .addTemporalMarker(6, () -> {
                    robot.stopIntake();
                })
                .addTemporalMarker(7.3, ()-> {
                    robot.autoCycleDepositPreset();
                })
                .addTemporalMarker(8, ()-> {
                    robot.smartClawOpen();
                })
                .waitSeconds(1.2)
                .build();
        TrajectorySequence cycle2 = drive.trajectorySequenceBuilder(centerCycle1.end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-13, -55.8), Math.toRadians(-90))
//                .addTemporalMarker(2, () -> {
//                    robot.autoIntake(3, 170);
//                })
                .setReversed(true)
                .addTemporalMarker(3.2, () -> {
                    robot.startIntake();
                })
                .addTemporalMarker(4.7, () -> {
                    robot.stopIntake();
                })
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-30, 47.7), Math.toRadians(90))

                .addTemporalMarker(5, () -> {
                    robot.startAutoIntake();
                    robot.claw.setClawOpen();
                })
                .addTemporalMarker(6, () -> {
                    robot.stopIntake();
                })
                .addTemporalMarker(7.3, ()-> {
                    robot.autoCycleDepositPreset();
                })
                .addTemporalMarker(8, ()-> {
                    robot.smartClawOpen();
                })
                .waitSeconds(1.2)
                .build();
        TrajectorySequence rightCycle1 = drive.trajectorySequenceBuilder(preloadBackdropRight.end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-13, -55.8), Math.toRadians(-90))
//                .addTemporalMarker(2, () -> {
//                    robot.autoIntake(3, 170);
//                })
                .setReversed(true)
                .addTemporalMarker(3.2, () -> {
                    robot.startIntake();
                })
                .addTemporalMarker(4.7, () -> {
                    robot.stopIntake();
                })
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-30, 47.7), Math.toRadians(90))

                .addTemporalMarker(5, () -> {
                    robot.startAutoIntake();
                    robot.claw.setClawOpen();
                })
                .addTemporalMarker(6, () -> {
                    robot.stopIntake();
                })
                .addTemporalMarker(7.3, ()-> {
                    robot.autoCycleDepositPreset();
                })
                .addTemporalMarker(8, ()-> {
                    robot.smartClawOpen();
                })
                .waitSeconds(1.2)
                .build();
        TrajectorySequence park = drive.trajectorySequenceBuilder(cycle2.end())
                .strafeRight(29)
                .back(12)
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
                drive.followTrajectorySequence(cycle2);
                drive.followTrajectorySequence(park);
                break;
            case LEFT:
                drive.followTrajectorySequence(preloadSpikeRight);
                drive.followTrajectorySequence(preloadBackdropRight);
                drive.followTrajectorySequence(rightCycle1);
                drive.followTrajectorySequence(cycle2);
                drive.followTrajectorySequence(park);
                break;
            case RIGHT:
                drive.followTrajectorySequence(preloadSpikeLeft);
                drive.followTrajectorySequence(preloadBackdropLeft);
                drive.followTrajectorySequence(leftCycle1);
                drive.followTrajectorySequence(cycle2);
                drive.followTrajectorySequence(park);
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