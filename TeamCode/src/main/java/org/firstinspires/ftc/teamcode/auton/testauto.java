package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
@Autonomous(group = "drive")

public class testauto extends LinearOpMode {
    SampleMecanumDrive drive;


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(59, -36, Math.PI);
        drive.setPoseEstimate(startPose);

        // PRELOAD PATHS
        TrajectorySequence preloadSpikeLeft = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineTo(new Vector2d(36, -40), Math.toRadians(235))
                .waitSeconds(1)
                .splineTo(new Vector2d(35, -60), Math.toRadians(270))
                .build();
        TrajectorySequence preloadBackdropLeft = drive.trajectorySequenceBuilder(preloadSpikeLeft.end())
                .splineTo(new Vector2d(10, -42.5), Math.toRadians(90))
                .splineTo(new Vector2d(10, 20), Math.toRadians(90))
                .splineTo(new Vector2d(29, 49), Math.toRadians(90))
                .waitSeconds(2)
                .build();
        TrajectorySequence leftToStack1 = drive.trajectorySequenceBuilder(preloadBackdropLeft.end())
                .setReversed(true)
                .splineTo(new Vector2d(10, 20), Math.toRadians(-90))
                .splineTo(new Vector2d(11, -61), Math.toRadians(-90))
                .waitSeconds(2)
                .build();

        TrajectorySequence preloadSpikeCenter = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .back(20)
                .forward(10)
                .setReversed(true)
                .splineTo(new Vector2d(-36, 49), Math.toRadians(90))

                .build();
        TrajectorySequence preloadBackdropCenter = drive.trajectorySequenceBuilder(preloadSpikeCenter.end())
                .setReversed(false)
                .splineTo(new Vector2d(10, -42.5), Math.toRadians(90))
                .splineTo(new Vector2d(10, 20), Math.toRadians(90))
                .splineTo(new Vector2d(29, 49), Math.toRadians(90))
                .waitSeconds(2)
                .build();
        TrajectorySequence centerToStack1 = drive.trajectorySequenceBuilder(preloadBackdropCenter.end())
                .setReversed(true)
                .splineTo(new Vector2d(10, 20), Math.toRadians(-90))
                .waitSeconds(2)
                .build();

        TrajectorySequence preloadSpikeRight = drive.trajectorySequenceBuilder(startPose)
                .forward(26)
                .turn(Math.toRadians(90))
                .waitSeconds(1)
                .strafeRight(21)
                .turn(Math.PI)
                .back(20)
                .waitSeconds(2)
                .build();
        TrajectorySequence preloadBackdropRight = drive.trajectorySequenceBuilder(preloadSpikeRight.end())
                //.setReversed(true)
                .splineToSplineHeading(new Pose2d(10, 20, Math.PI/2), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(29, 49, Math.PI/2), Math.toRadians(90))
                .waitSeconds(2)
                .build();
        TrajectorySequence rightToStack1 = drive.trajectorySequenceBuilder(preloadBackdropRight.end())
                .setReversed(true)
                .splineTo(new Vector2d(10, 20), Math.toRadians(-90))
                .splineTo(new Vector2d(11, -61), Math.toRadians(-90))
                .waitSeconds(2)
                .build();

        // SHARED PATHS
        TrajectorySequence stackToBackdrop1 = drive.trajectorySequenceBuilder(leftToStack1.end())
                .setReversed(true)
                .splineTo(new Vector2d(10, 20), Math.toRadians(90))
                .splineTo(new Vector2d(36, 49), Math.toRadians(90))
                .waitSeconds(2)
                .build();

        TrajectorySequence backdropToStack2 = drive.trajectorySequenceBuilder(stackToBackdrop1.end())
                .setReversed(false)
                .splineTo(new Vector2d(10, 20), Math.toRadians(-90))
                .splineTo(new Vector2d(11, -61), Math.toRadians(-90))
                .waitSeconds(2)
                .build();
        TrajectorySequence stackToBackdrop2 = drive.trajectorySequenceBuilder(backdropToStack2.end())
                .setReversed(true)
                .splineTo(new Vector2d(10, 20), Math.toRadians(90))
                .splineTo(new Vector2d(36, 49), Math.toRadians(90))
                .waitSeconds(2)
                .build();

        TrajectorySequence backdropToStack3 = drive.trajectorySequenceBuilder(stackToBackdrop2.end())
                .setReversed(false)
                .splineTo(new Vector2d(10, 20), Math.toRadians(-90))
                .splineTo(new Vector2d(11, -61), Math.toRadians(-90))
                .waitSeconds(2)
                .build();
        TrajectorySequence stackToBackdrop3 = drive.trajectorySequenceBuilder(backdropToStack3.end())
                .setReversed(true)
                .splineTo(new Vector2d(10, 20), Math.toRadians(90))
                .splineTo(new Vector2d(36, 49), Math.toRadians(90))
                .waitSeconds(2)
                .build();

        TrajectorySequence park = drive.trajectorySequenceBuilder(stackToBackdrop3.end())
                .strafeLeft(10)
                .build();


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
                drive.followTrajectorySequence(centerToStack1);
                break;
            case LEFT:
                drive.followTrajectorySequence(preloadSpikeLeft);
                drive.followTrajectorySequence(preloadBackdropLeft);
                drive.followTrajectorySequence(leftToStack1);
                break;
            case RIGHT:
                drive.followTrajectorySequence(preloadSpikeRight);
                drive.followTrajectorySequence(preloadBackdropCenter);
                drive.followTrajectorySequence(rightToStack1);
                break;
        }

        drive.followTrajectorySequence(stackToBackdrop1);
        drive.followTrajectorySequence(backdropToStack2);
        drive.followTrajectorySequence(stackToBackdrop2);
        drive.followTrajectorySequence(backdropToStack3);
        drive.followTrajectorySequence(stackToBackdrop3);
        drive.followTrajectorySequence(park);



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
