package org.firstinspires.ftc.teamcode.auton;

import android.util.Size;

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
import org.firstinspires.ftc.teamcode.subsystems.vision.YoinkElementCVProcessor;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(group = "drive")

public class RedBackdropSidePreload extends LinearOpMode {
    Robot robot;
    VisionPortal visionPortal;
    YoinkElementCVProcessor teamElementProcessor;
    YoinkElementCVProcessor.PropLocation propLocation = YoinkElementCVProcessor.PropLocation.UNFOUND;
    @Override
    public void runOpMode() throws InterruptedException {

        teamElementProcessor = new YoinkElementCVProcessor(AllianceColor.RED, telemetry);
//        teamElementProcessor.alliance = AllianceColor.RED;
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // the camera on your robot is named "Webcam 1" by default
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .addProcessor(teamElementProcessor)

                .build();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        robot = new Robot(hardwareMap, true);
        Pose2d startPose = new Pose2d(62, 13, Math.toRadians(0));
        robot.autoIntake();

        drive.setPoseEstimate(startPose);

        // PRELOAD PATHS
        TrajectorySequence preloadSpikeLeft = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(30, 10, Math.toRadians(90)), Math.toRadians(0))
                .forward(15)
                .turn(Math.toRadians(180))
                .build();

        TrajectorySequence preloadBackdropLeft = drive.trajectorySequenceBuilder(preloadSpikeLeft.end())
                .setReversed(true)
                .back(25)
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
                .strafeRight(20)
                .back(10)
                .build();

        TrajectorySequence preloadSpikeCenter = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .back(29.5)
                .forward(14)
                .build();

        TrajectorySequence preloadBackdropCenter = drive.trajectorySequenceBuilder(preloadSpikeCenter.end())
                .setReversed(true)
                .splineTo(new Vector2d(34, 49.5), Math.toRadians(90))
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
                .strafeRight(22)
                .back(10)
                .build();

        TrajectorySequence preloadSpikeRight = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(38, 25), Math.toRadians(180))
                .forward(15)
                .turn(Math.toRadians(-90))
                .build();

        TrajectorySequence preloadBackdropRight = drive.trajectorySequenceBuilder(preloadSpikeRight.end())
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(42, 50, Math.toRadians(270)), Math.toRadians(90))
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
                .strafeRight(35)
                .back(10)
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
        propLocation = teamElementProcessor.getLocation();
//        telemetry.addData("Camera State", visionPortal.getCameraState());
//        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
//            telemetry.addData("Camera State", visionPortal.getCameraState());

            if (propLocation == YoinkElementCVProcessor.PropLocation.UNFOUND) {
//                telemetry.addLine("Team Element Location: <b>NOT FOUND</b>");
            } else {
//                telemetry.addData("Team Element Location", propLocation);
            }
//            telemetry.update();
        }


        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */



        waitForStart();

        if (isStopRequested()) return;

        // shuts down the camera once the match starts, we dont need to look any more

        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.close();
        }

        propLocation = teamElementProcessor.getLocation();

//        // if it is UNFOUND, you can manually set it to any of the other positions to guess
        if (propLocation == YoinkElementCVProcessor.PropLocation.UNFOUND) {
            propLocation = YoinkElementCVProcessor.PropLocation.CENTER;
        }

        robot.slides.launchAsThread(telemetry);
        switch (propLocation) {
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

//        drive.followTrajectorySequence(park);



        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = drive.getPoseEstimate();

        robot.slides.destroyThreads(telemetry);
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
