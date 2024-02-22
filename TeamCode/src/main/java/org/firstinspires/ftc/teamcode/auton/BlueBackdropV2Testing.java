package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
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
import org.firstinspires.ftc.teamcode.subsystems.vision.CVMaster;
import org.opencv.core.Scalar;

@Config
@Autonomous(group = "drive")

public class BlueBackdropV2Testing extends LinearOpMode {
    Robot robot;
    CVMaster cv;


    @Override
    public void runOpMode() throws InterruptedException {
        Scalar lower = new Scalar(103, 120, 50); // the lower hsv threshold for your detection
        Scalar upper = new Scalar(130, 255, 250); // the upper hsv threshold for your detection
        double minArea = 3000; // the minimum area for the detection to consider for your prop

        cv = new CVMaster(hardwareMap);
        cv.initProp(AllianceColor.BLUE);

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

                .addTemporalMarker(0.7, () -> {
                    //intake
                })
                .addTemporalMarker(2.1, () -> {
                    //robot.stopIntake();
                })
                .build();
        TrajectorySequence wait = drive.trajectorySequenceBuilder(preloadSpikeLeft.end())
                .back(3)
                .addTemporalMarker(0, () ->{
                    robot.farPos();
                    robot.intake.runToPreset(Levels.INTAKE);
                })
                .waitSeconds(3)
                .build();

        TrajectorySequence preloadBackdropLeft = drive.trajectorySequenceBuilder(preloadSpikeLeft.end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-8, 12), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-35, 12), Math.toRadians(90))
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
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-8, 12), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-35, 12), Math.toRadians(90))
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
                .addTemporalMarker(3.7, () -> {
                    //intake
                })
                .addTemporalMarker(5.2, () -> {
                    //robot.stopIntake();
                })
                .build();


        TrajectorySequence preloadBackdropRight = drive.trajectorySequenceBuilder(preloadSpikeRight.end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-8, 12), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-35, 12), Math.toRadians(90))
                .waitSeconds(1)
                .build();
        TrajectorySequence l1 = drive.trajectorySequenceBuilder(preloadBackdropLeft.end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-40, 48), Math.toRadians(90))
                .addTemporalMarker(3, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(4, () -> {
                    robot.smartClawOpen();
                })
                .waitSeconds(3)
                .build();
        TrajectorySequence l2 = drive.trajectorySequenceBuilder(preloadBackdropLeft.end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-37, 48), Math.toRadians(90))
                .addTemporalMarker(3, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(4, () -> {
                    robot.smartClawOpen();
                })
                .waitSeconds(3)
                .build();
        TrajectorySequence c1 = drive.trajectorySequenceBuilder(preloadBackdropLeft.end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-34, 48), Math.toRadians(90))
                .addTemporalMarker(3, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(4, () -> {
                    robot.smartClawOpen();
                })
                .waitSeconds(3)
                .build();
        TrajectorySequence c2 = drive.trajectorySequenceBuilder(preloadBackdropLeft.end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-31, 48), Math.toRadians(90))
                .addTemporalMarker(3, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(4, () -> {
                    robot.smartClawOpen();
                })
                .waitSeconds(3)
                .build();
        TrajectorySequence r1 = drive.trajectorySequenceBuilder(preloadBackdropLeft.end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-28, 48), Math.toRadians(90))
                .addTemporalMarker(3, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(4, () -> {
                    robot.smartClawOpen();
                })
                .waitSeconds(3)
                .build();
        TrajectorySequence r2 = drive.trajectorySequenceBuilder(preloadBackdropLeft.end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-25, 48), Math.toRadians(90))
                .addTemporalMarker(3, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(4, () -> {
                    robot.smartClawOpen();
                })
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

        recordedPropPosition = YoinkP2Pipeline.PropPositions.RIGHT;
        // shuts down the camera once the match starts, we dont need to look any more
        cv.switchToAuton(AllianceColor.BLUE);
        cv.preloadProcessor.setTargetAprilTagID(recordedPropPosition);

        robot.launchSubsystemThread(telemetry);
        switch (recordedPropPosition) {
            case CENTER:
                drive.followTrajectorySequence(preloadSpikeCenter);
                drive.followTrajectorySequence(wait);
                drive.followTrajectorySequence(preloadBackdropCenter);
                telemetry.addData("yo", cv.detectPreload());
                if (cv.detectPreload() == YoinkP2Pipeline.PropPositions.LEFT){
                    telemetry.addData("side: ", "left");
                    drive.followTrajectorySequence(c2);
                } else{
                    telemetry.addData("side: ", "right/none");
                    drive.followTrajectorySequence(c1);
                }
                telemetry.update();
//                drive.followTrajectorySequence(centerCycle1);
//                drive.followTrajectorySequence(centerCycle2);
                break;
            case LEFT:
                drive.followTrajectorySequence(preloadSpikeRight);
                drive.followTrajectorySequence(wait);
                drive.followTrajectorySequence(preloadBackdropRight);
                if (cv.detectPreload() == YoinkP2Pipeline.PropPositions.LEFT){
                    telemetry.addData("side: ", "left");
                    drive.followTrajectorySequence(l2);
                } else{
                    telemetry.addData("side: ", "right/none");
                    drive.followTrajectorySequence(l1);
                }
                telemetry.update();
                //drive.followTrajectorySequence(rightCycle1);
                break;
            case RIGHT:
                drive.followTrajectorySequence(preloadSpikeLeft);
                drive.followTrajectorySequence(wait);
                drive.followTrajectorySequence(preloadBackdropLeft);
                if (cv.detectPreload() == YoinkP2Pipeline.PropPositions.LEFT){
                    telemetry.addData("side: ", "left");
                    drive.followTrajectorySequence(r2);
                } else{
                    telemetry.addData("side: ", "right/none ");
                    drive.followTrajectorySequence(r1);
                }
                telemetry.update();
                //drive.followTrajectorySequence(leftCycle1);
                break;
        }

//        drive.followTrajectorySequence(park);



        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = drive.getPoseEstimate();

        robot.destroyThreads(telemetry);
        cv.kill();

        while (!isStopRequested() && opModeIsActive()) ;
    }

    public static double rad(double degrees) {
        return Math.toRadians(degrees);
    }

    public static double in(double centimeters) {
        return centimeters * 0.3837008;
    }
}


