package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.lib.AllianceColor;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.vision.CVMaster;
import org.firstinspires.ftc.teamcode.subsystems.vision.YoinkP2Pipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group = "drive")

public class BlueSpikeFar extends LinearOpMode {
    Robot robot;
    CVMaster cv;
    double waitt = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        cv = new CVMaster(hardwareMap);
        cv.initProp(AllianceColor.BLUE);

        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        robot = new Robot(hardwareMap, true);
        Pose2d startPose = new Pose2d(-62, -36, Math.toRadians(180));
        robot.initPos();

        drive.setPoseEstimate(startPose);

        // PRELOAD PATHS
        TrajectorySequence wait = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(waitt)
                .build();
        TrajectorySequence preloadSpikeRight = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-34, -33, Math.toRadians(235)), Math.toRadians(30))
                .forward(12)
                .turn(Math.toRadians(35))
                .build();

        TrajectorySequence preloadBackdropRight = drive.trajectorySequenceBuilder(preloadSpikeRight.end())
                .strafeRight(6)
                //.splineToLinearHeading(new Pose2d(-48, -40, Math.toRadians(235)), Math.toRadians(30))
                //.splineToLinearHeading(new Pose2d(-57, -40, Math.toRadians(-90)), Math.toRadians(-90))
                //.setReversed(false)
                //.splineToConstantHeading(new Vector2d(-57, -25), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-57, 14), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-39, 48.8), Math.toRadians(90))
                .addTemporalMarker(0, () -> {
                    this.robot.intake.setAngle(120);
                })
                .addTemporalMarker(3, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(4, () -> {
                    robot.smartClawOpen();
                })
                .waitSeconds(3)
                .build();

        TrajectorySequence preloadSpikeCenter = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .back(28)
                .forward(24)
                .turn(Math.PI/2)
                .build();

        TrajectorySequence preloadBackdropCenter = drive.trajectorySequenceBuilder(preloadSpikeCenter.end())
                .splineToConstantHeading(new Vector2d(-59, -2), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-35, 48.8), Math.toRadians(90))
                .addTemporalMarker(0, () -> {
                    this.robot.intake.setAngle(120);
                })
                .addTemporalMarker(3.9, () -> {
                    robot.autoPreloadDepositPreset();
                })
                .addTemporalMarker(4.9, () -> {
                    robot.smartClawOpen();
                })
                .waitSeconds(4)
                .build();
        TrajectorySequence parkCenter = drive.trajectorySequenceBuilder(preloadBackdropCenter.end())
                .strafeRight(28)
                .back(11)
                .build();

        TrajectorySequence preloadSpikeLeft = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(-38, -47), Math.toRadians(0))
                .forward(17)
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence preloadBackdropLeft = drive.trajectorySequenceBuilder(preloadSpikeLeft.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-58, -8, Math.toRadians(270)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-29.5, 48.8, Math.toRadians(270)), Math.toRadians(90))
                //.splineToLinearHeading(new Pose2d(-42, 52, Math.toRadians(270)), Math.toRadians(90))
                //                .splineToConstantHeading(new Vector2d(-29, 52), Math.toRadians(90))
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
        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(preloadBackdropLeft.end())
                .strafeRight(29)
                .back(11)
                .build();
        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(preloadBackdropRight.end())
                .strafeRight(23)
                .back(11)
                .build();

//        TrajectorySequence cycle1 = drive.trajectorySequenceBuilder(preloadBackdropCenter.end())
//                .setReversed(false)
//                .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(11, -58), Math.toRadians(-90))
//                .addTemporalMarker(2, () -> {
//                    robot.autoIntake(3, 170);
//                })
//                .setReversed(true)
//                .waitSeconds(4)
//                .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(34, 49), Math.toRadians(90))
//                .build();


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

        // shuts down the camera once the match starts, we dont need to look any more
        cv.switchToAprilTags();


        // gets the recorded prop position
        YoinkP2Pipeline.PropPositions recordedPropPosition = cv.colourMassDetectionProcessor.getRecordedPropPosition();

        // now we can use recordedPropPosition to determine where the prop is! if we never saw a prop, your recorded position will be UNFOUND.
        // if it is UNFOUND, you can manually set it to any of the other positions to guess
        if (recordedPropPosition == YoinkP2Pipeline.PropPositions.UNFOUND) {
            recordedPropPosition = YoinkP2Pipeline.PropPositions.CENTER;
        }
        robot.launchSubsystemThread(telemetry);
        switch (recordedPropPosition) {
            case CENTER:
                drive.followTrajectorySequence(preloadSpikeCenter);
                drive.followTrajectorySequence(wait);
                drive.followTrajectorySequence(preloadBackdropCenter);
                drive.followTrajectorySequence(parkCenter);
                break;
            case LEFT:
                drive.followTrajectorySequence(preloadSpikeLeft);
                drive.followTrajectorySequence(wait);
                drive.followTrajectorySequence(preloadBackdropLeft);
                drive.followTrajectorySequence(parkLeft);
                break;
            case RIGHT:
                drive.followTrajectorySequence(preloadSpikeRight);
                drive.followTrajectorySequence(wait);
                drive.followTrajectorySequence(preloadBackdropRight);
                drive.followTrajectorySequence(parkRight);
                break;
        }

//        drive.followTrajectorySequence(park);



        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = drive.getPoseEstimate();

        robot.slides.destroyThreads(telemetry);
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