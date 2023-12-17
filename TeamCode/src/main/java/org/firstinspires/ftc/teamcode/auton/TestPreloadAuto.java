package org.firstinspires.ftc.teamcode.auton;

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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(group = "drive")

public class TestPreloadAuto extends LinearOpMode {
    SampleMecanumDrive drive;


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(62, 12, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        // PRELOAD PATHS
        TrajectorySequence preloadSpikeLeft = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(45, 20), Math.toRadians(15))
                .strafeRight(10)
                .build();

        TrajectorySequence preloadBackdropLeft = drive.trajectorySequenceBuilder(preloadSpikeLeft.end())
                .setReversed(true)
                .splineTo(new Vector2d(29, 49), Math.toRadians(90))
                .waitSeconds(2)
                .build();

        TrajectorySequence preloadSpikeCenter = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .back(29.5)
                .forward(14)
                .build();

        TrajectorySequence preloadBackdropCenter = drive.trajectorySequenceBuilder(preloadSpikeCenter.end())
                .setReversed(true)
                .splineTo(new Vector2d(34, 49), Math.toRadians(90))
                .waitSeconds(2)
                .build();

        TrajectorySequence preloadSpikeRight = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(43, 8), Math.toRadians(-15))
                .forward(10)
                .build();

        TrajectorySequence preloadBackdropRight = drive.trajectorySequenceBuilder(preloadSpikeRight.end())
                .setReversed(true)
                .splineTo(new Vector2d(40, 49), Math.toRadians(90))
                .waitSeconds(2)
                .build();

        TrajectorySequence cycle1 = drive.trajectorySequenceBuilder(preloadBackdropCenter.end())
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(11, -61), Math.toRadians(-90))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(10, 20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(34, 49), Math.toRadians(90))
                .build();


//        TrajectorySequence park = drive.trajectorySequenceBuilder(prel.end())
//                .strafeLeft(10)
//                .build();


        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {

        }


        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        // shuts down the camera once the match starts, we dont need to look any more

        TeamElementCVProcessor.Location propLocation = TeamElementCVProcessor.Location.CENTER;


        waitForStart();

        if (isStopRequested()) return;

        switch (propLocation) {
            case CENTER:
                drive.followTrajectorySequence(preloadSpikeCenter);
                drive.followTrajectorySequence(preloadBackdropCenter);
                drive.followTrajectorySequence(cycle1);
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

        while (!isStopRequested() && opModeIsActive()) ;
    }

    public static double rad(double degrees) {
        return Math.toRadians(degrees);
    }

    public static double in(double centimeters) {
        return centimeters * 0.3837008;
    }
}
