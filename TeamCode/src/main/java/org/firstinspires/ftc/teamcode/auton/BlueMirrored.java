//package org.firstinspires.ftc.teamcode.auton;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
//import org.firstinspires.ftc.teamcode.lib.AllianceColor;
//import org.firstinspires.ftc.teamcode.lib.PoseStorage;
//import org.firstinspires.ftc.teamcode.subsystems.vision.YoinkP2Pipeline;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.opencv.core.Scalar;
//
//public class BlueMirrored extends LinearOpMode{
//    /** BACKDROP SIDE **/
//    Robot robot;
//    private VisionPortal visionPortal;
//    private YoinkP2Pipeline colourMassDetectionProcessor;
//    AprilTagProcessor processor;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
//        robot = new Robot(hardwareMap, true);
//        Pose2d startPose = new Pose2d(-62, 13, Math.toRadians(0));
//        robot.initPos();
//
//        drive.setPoseEstimate(startPose);
//
//        // PRELOAD PATHS
//        TrajectorySequence preloadSpikeLeft = drive.trajectorySequenceBuilder(startPose)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-38, 11.7, Math.toRadians(235)), Math.toRadians(30))
//                .build();
//
//        TrajectorySequence preloadBackdropLeft = drive.trajectorySequenceBuilder(preloadSpikeLeft.end())
//                .setReversed(false)
//                .splineToLinearHeading(new Pose2d(-30, 49, Math.toRadians(270)), Math.toRadians(90))
//                .addTemporalMarker(0, () -> {
//                    this.robot.intake.setAngle(120);
//                })
//                .addTemporalMarker(1.5, () -> {
//                    robot.autoPreloadDepositPreset();
//                })
//                .addTemporalMarker(2.3, () -> {
//                    robot.smartClawOpen();
//                })
//                .waitSeconds(0.8)
//                .build();
//
//        TrajectorySequence preloadSpikeCenter = drive.trajectorySequenceBuilder(startPose)
//                .setReversed(true)
//                .splineTo(new Vector2d(-40, 13), Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(-48, 13, Math.toRadians(180)), Math.toRadians(0))
//                .build();
//
//        TrajectorySequence preloadBackdropCenter = drive.trajectorySequenceBuilder(preloadSpikeCenter.end())
//                .setReversed(true)
//                .splineTo(new Vector2d(-32.5, 50.5), Math.toRadians(90))
//                .addTemporalMarker(0, () -> {
//                    this.robot.intake.setAngle(120);
//                })
//                .addTemporalMarker(1.1, () -> {
//                    robot.autoPreloadDepositPreset();
//                })
//                .addTemporalMarker(1.95, () -> {
//                    robot.smartClawOpen();
//                })
//                .waitSeconds(0.2)
//                .build();
//
//        TrajectorySequence preloadSpikeRight = drive.trajectorySequenceBuilder(startPose)
//                .setReversed(true)
//                .splineTo(new Vector2d(-38, 26), Math.toRadians(0))
//                .forward(15)
//                .build();
//
//        TrajectorySequence preloadBackdropRight = drive.trajectorySequenceBuilder(preloadSpikeRight.end())
//                .splineToLinearHeading(new Pose2d(-42, 48, Math.toRadians(270)), Math.toRadians(90))
//                .addTemporalMarker(0, () -> {
//                    this.robot.intake.setAngle(120);
//                })
//                .addTemporalMarker(0.8, () -> {
//                    robot.autoPreloadDepositPreset();
//                })
//                .addTemporalMarker(1.7, () -> {
//                    robot.smartClawOpen();
//                })
//                .waitSeconds(0.2)
//                .build();
//
//        TrajectorySequence centerCycle1 = drive.trajectorySequenceBuilder(preloadBackdropCenter.end())
//                .setReversed(false)
//                .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(-13, -55.8), Math.toRadians(-90))
////                .addTemporalMarker(2, () -> {
////                    robot.autoIntake(3, 170);
////                })
//                .setReversed(true)
//                .addTemporalMarker(2.3, () -> {
//                    robot.startIntake();
//                })
//                .strafeRight(4)
//                .forward(2)
//                .strafeLeft(8)
//                .back(2)
//                .addTemporalMarker(4.5, () -> {
//                    robot.intake.setAngle(90);
//                })
//                .addTemporalMarker(4.5, () -> {
//                    robot.intake.reverseIntake();
//                })
//                .addTemporalMarker(4.7, () -> {
//                    robot.stopIntake();
//                })
//                .waitSeconds(0.2)
//                .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(-30, 47.7), Math.toRadians(90))
//                .addTemporalMarker(6.5, () -> {
//                    robot.startAutoIntake();
//                    robot.claw.setClawOpen();
//                })
//                .addTemporalMarker(7, () -> {
//                    robot.stopIntake();
//                })
//                .addTemporalMarker(8.2, () -> {
//                    robot.autoCycleDepositPreset();
//                })
//                .addTemporalMarker(9.5, () -> {
//                    robot.smartClawOpen();
//                })
//                .waitSeconds(1)
//                .build();
//
//        TrajectorySequence centerCycle2 = drive.trajectorySequenceBuilder(centerCycle1.end())
//                .setReversed(false)
//                .splineToConstantHeading(new Vector2d(-7, 20), Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(-12, -57.2), Math.toRadians(-90))
////                .addTemporalMarker(2, () -> {
////                    robot.autoIntake(3, 170);
////                })
//                .setReversed(true)
//                .addTemporalMarker(2.3, () -> {
//                    robot.startIntake();
//                })
//                .strafeRight(4)
//                .forward(2)
//                .strafeLeft(8)
//                .back(2)
//                .addTemporalMarker(4.5, () -> {
//                    robot.intake.setAngle(90);
//                })
//                .addTemporalMarker(4.5, () -> {
//                    robot.intake.reverseIntake();
//                })
//                .back(2)
//                .addTemporalMarker(4.7, () -> {
//                    robot.stopIntake();
//                })
//                .waitSeconds(0.2)
//                .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(-30, 47.8), Math.toRadians(90))
//                .addTemporalMarker(6.5, () -> {
//                    robot.startAutoIntake();
//                    robot.claw.setClawOpen();
//                })
//                .addTemporalMarker(7, () -> {
//                    robot.stopIntake();
//                })
//                .addTemporalMarker(8.5, () -> {
//                    robot.autoCycleDepositPreset();
//                })
//                .addTemporalMarker(9.5, () -> {
//                    robot.smartClawOpen();
//                })
//                .addTemporalMarker(13, () -> {
//                    robot.slides.runToPosition(0);
//                })
//                .waitSeconds(1)
//                .strafeLeft(21)
//                .back(7)
//                .waitSeconds(2)
//                .build();
//
//        TrajectorySequence leftCycle1 = drive.trajectorySequenceBuilder(preloadBackdropLeft.end())
//                .setReversed(false)
//                .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(-13, -56.1), Math.toRadians(-90))
////                .addTemporalMarker(2, () -> {
////                    robot.autoIntake(3, 170);
////                })
//                .setReversed(true)
//                .addTemporalMarker(2.5, () -> {
//                    robot.startIntake();
//                })
//                .strafeRight(4)
//                .forward(2)
//                .strafeLeft(8)
//                .back(2)
//                .addTemporalMarker(4.5, () -> {
//                    robot.intake.setAngle(90);
//                })
//                .addTemporalMarker(4.5, () -> {
//                    robot.intake.reverseIntake();
//                })
//                .addTemporalMarker(4.7, () -> {
//                    robot.stopIntake();
//                })
//                .waitSeconds(0.2)
//                .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(-30, 48), Math.toRadians(90))
//                .addTemporalMarker(6.5, () -> {
//                    robot.startAutoIntake();
//                    robot.claw.setClawOpen();
//                })
//                .addTemporalMarker(7, () -> {
//                    robot.stopIntake();
//                })
//                .addTemporalMarker(8.2, () -> {
//                    robot.autoCycleDepositPreset();
//                })
//                .addTemporalMarker(9.5, () -> {
//                    robot.smartClawOpen();
//                })
//                .waitSeconds(1)
//                .build();
//
//        TrajectorySequence leftCycle2 = drive.trajectorySequenceBuilder(leftCycle1.end())
//                .setReversed(false)
//                .splineToConstantHeading(new Vector2d(-7, 20), Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(-12, -57), Math.toRadians(-90))
////                .addTemporalMarker(2, () -> {
////                    robot.autoIntake(3, 170);
////                })
//                .setReversed(true)
//                .addTemporalMarker(2.5, () -> {
//                    robot.startIntake();
//                })
//                .strafeRight(4)
//                .forward(2)
//                .strafeLeft(8)
//                .back(2)
//                .addTemporalMarker(4.5, () -> {
//                    robot.intake.setAngle(90);
//                })
//                .addTemporalMarker(4.5, () -> {
//                    robot.intake.reverseIntake();
//                })
//                .back(2)
//                .addTemporalMarker(4.7, () -> {
//                    robot.stopIntake();
//                })
//                .waitSeconds(0.2)
//                .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(-30, 47.65), Math.toRadians(90))
//                .addTemporalMarker(6.5, () -> {
//                    robot.startAutoIntake();
//                    robot.claw.setClawOpen();
//                })
//                .addTemporalMarker(7, () -> {
//                    robot.stopIntake();
//                })
//                .addTemporalMarker(8.5, () -> {
//                    robot.autoCycleDepositPreset();
//                })
//                .addTemporalMarker(9.7, () -> {
//                    robot.smartClawOpen();
//                })
//                .addTemporalMarker(13, () -> {
//                    robot.slides.runToPosition(0);
//                })
//                .waitSeconds(1)
//                .strafeLeft(21)
//                .back(7)
//                .waitSeconds(2)
//                .build();
//
//        TrajectorySequence rightCycle1 = drive.trajectorySequenceBuilder(preloadBackdropRight.end())
//                .setReversed(false)
//                .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(-13, -54), Math.toRadians(-90))
////                .addTemporalMarker(2, () -> {
////                    robot.autoIntake(3, 170);
////                })
//                .setReversed(true)
//                .addTemporalMarker(2.3, () -> {
//                    robot.startIntake();
//                })
//                .strafeRight(4)
//                .forward(2)
//                .strafeLeft(8)
//                .back(2)
//                .addTemporalMarker(4.5, () -> {
//                    robot.intake.setAngle(90);
//                })
//                .addTemporalMarker(4.5, () -> {
//                    robot.intake.reverseIntake();
//                })
//                .addTemporalMarker(4.7, () -> {
//                    robot.stopIntake();
//                })
//                .waitSeconds(0.2)
//                .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(-30, 48), Math.toRadians(90))
//                .addTemporalMarker(6.5, () -> {
//                    robot.startAutoIntake();
//                    robot.claw.setClawOpen();
//                })
//                .addTemporalMarker(7, () -> {
//                    robot.stopIntake();
//                })
//                .addTemporalMarker(8.2, () -> {
//                    robot.autoCycleDepositPreset();
//                })
//                .addTemporalMarker(9.5, () -> {
//                    robot.smartClawOpen();
//                })
//                .waitSeconds(1)
//                .build();
//
//        TrajectorySequence rightCycle2 = drive.trajectorySequenceBuilder(rightCycle1.end()).setReversed(false)
//                .splineToConstantHeading(new Vector2d(-7, 20), Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(-12, -57.2), Math.toRadians(-90))
////                .addTemporalMarker(2, () -> {
////                    robot.autoIntake(3, 170);
////                })
//                .setReversed(true)
//                .addTemporalMarker(2.3, () -> {
//                    robot.startIntake();
//                })
//                .strafeRight(4)
//                .forward(2)
//                .strafeLeft(8)
//                .back(2)
//                .addTemporalMarker(4.5, () -> {
//                    robot.intake.setAngle(90);
//                })
//                .addTemporalMarker(4.5, () -> {
//                    robot.intake.reverseIntake();
//                })
//                .back(2)
//                .addTemporalMarker(4.7, () -> {
//                    robot.stopIntake();
//                })
//                .waitSeconds(0.2)
//                .splineToConstantHeading(new Vector2d(-10, 20), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(-30, 47.8), Math.toRadians(90))
//                .addTemporalMarker(6.5, () -> {
//                    robot.startAutoIntake();
//                    robot.claw.setClawOpen();
//                })
//                .addTemporalMarker(7, () -> {
//                    robot.stopIntake();
//                })
//                .addTemporalMarker(8.5, () -> {
//                    robot.autoCycleDepositPreset();
//                })
//                .addTemporalMarker(9.5, () -> {
//                    robot.smartClawOpen();
//                })
//                .addTemporalMarker(13, () -> {
//                    robot.slides.runToPosition(0);
//                })
//                .waitSeconds(1)
//                .strafeLeft(21)
//                .back(7)
//                .waitSeconds(2)
//                .build();
//
//        /** FAR **/
//        // PRELOAD PATHS
//        TrajectorySequence preloadSpikeLeft = drive.trajectorySequenceBuilder(startPose)
//                .setReversed(true)
//                .addTemporalMarker(0, () -> {
//                    this.robot.farPos();
//                })
//                .splineTo(new Vector2d(-38, -42), Math.toRadians(0))
//                .forward(22)
//                .turn(Math.toRadians(90))
//                //.strafeLeft(2)
//                .build();
//
//        TrajectorySequence preloadBackdropLeft = drive.trajectorySequenceBuilder(preloadSpikeLeft.end())
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(-58, 20, Math.toRadians(-90)), Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(-26, 49, Math.toRadians(-90)), Math.toRadians(90))
//                .addTemporalMarker(0, () -> {
//                    this.robot.intake.setAngle(120);
//                })
//                .addTemporalMarker(2.6, () -> {
//                    robot.autoPreloadDepositPreset();
//                })
//                .addTemporalMarker(3.6, () -> {
//                    robot.smartClawOpen();
//                })
//                .waitSeconds(2)
//                .strafeRight(31)
//                .back(10)
//                .build();
//
//        TrajectorySequence preloadSpikeCenter = drive.trajectorySequenceBuilder(startPose)
//                .setReversed(true)
//                .addTemporalMarker(0, () -> {
//                    this.robot.farPos();
//                })
//                .back(29)
//                .forward(25.5)
//                .turn(Math.toRadians(90))
//                .strafeRight(2)
//                .build();
//
//        TrajectorySequence preloadBackdropCenter = drive.trajectorySequenceBuilder(preloadSpikeCenter.end())
//                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(-59, 20), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(-32, 49), Math.toRadians(90))
//                .addTemporalMarker(0, () -> {
//                    this.robot.intake.setAngle(120);
//                })
//                .addTemporalMarker(2.6, () -> {
//                    robot.autoPreloadDepositPreset();
//                })
//                .addTemporalMarker(3.6, () -> {
//                    robot.smartClawOpen();
//                })
//                .waitSeconds(2)
//                .strafeRight(26)
//                .back(10)
//                .build();
//
//        TrajectorySequence preloadSpikeRight = drive.trajectorySequenceBuilder(startPose)
//                .setReversed(true)
//                .addTemporalMarker(0, () -> {
//                    this.robot.farPos();
//                })
//                .splineTo(new Vector2d(-38, -27), Math.toRadians(45))
//                .forward(17)
//                .turn(Math.toRadians(45))
//                .build();
//
//        TrajectorySequence preloadBackdropRight = drive.trajectorySequenceBuilder(preloadSpikeRight.end())
//                .strafeRight(10)
//                .back(20)
//                .splineToConstantHeading(new Vector2d(-59, 20), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(-38, 49), Math.toRadians(90))
//                .addTemporalMarker(0, () -> {
//                    this.robot.intake.setAngle(120);
//                })
//                .addTemporalMarker(3, () -> {
//                    robot.autoPreloadDepositPreset();
//                })
//                .addTemporalMarker(4, () -> {
//                    robot.smartClawOpen();
//                })
//                .waitSeconds(2)
//                .strafeRight(20)
//                .back(10)
//                .build();
//
//    }
//}
