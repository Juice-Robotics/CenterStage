package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.AllianceColor;
import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.concurrent.TimeUnit;

@TeleOp(group = "competition")
public class TeleOpSafe extends LinearOpMode {
    // Define 2 states, driver control or alignment control
    enum Mode {
        NORMAL_CONTROL,
        ALIGN_TO_POINT
    }
    private Mode currentMode = Mode.NORMAL_CONTROL;

    // Declare a PIDF Controller to regulate heading
    // Use the same gains as SampleMecanumDrive's heading controller
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize your own robot class
        Robot robot = new Robot(hardwareMap,false);

        robot.drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.drive.getLocalizer().setPoseEstimate(PoseStorage.currentPose);
        headingController.setInputBounds(-Math.PI, Math.PI);

        double x;
        double y;
        double rx;

//        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.experimental.setMaximumParallelCommands(4);
//        PhotonCore.enable();
//        PhotonCore.start(hardwareMap);

        ElapsedTime matchTimer;

        int buzzers = 0;

        boolean autoCloseEnabled = true;
        boolean autoClosePreviousState = false;
        boolean previousDpadLeftState = false;
        boolean previousDpadRightState = false;
        boolean previousDroneState = false;
        boolean previousIntakeState = false;
        boolean previousAutoAlignState = false;
        boolean isPressed = false;
        double intakePreviousPos;
        float previousLeftTriggerState = 0;
        boolean previousSquare = false;
        boolean previousCircle = false;


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;
        matchTimer = new ElapsedTime();
        //intakePreviousPos = robot.intake.intakeMotor.getCurrentPosition();
        robot.slides.runToPosition(0);
        robot.slides.resetAllEncoders();
        robot.drone.prime();

        while (opModeIsActive() && !isStopRequested()) {
//
//            if (gamepad1.dpad_up) {
//                robot.slides.resetAllEncoders();
//            }
            //intakePreviousPos = robot.checkJam(intakePreviousPos);
            //DRIVE
            switch (currentMode) {
                case NORMAL_CONTROL:
                    if (gamepad1.right_trigger > 0.5) {
                        x = -gamepad1.left_stick_x * (1 - 0.66 * gamepad1.right_trigger);
                        y = -gamepad1.left_stick_y * (1 - 0.66 * gamepad1.right_trigger);
                        rx = gamepad1.right_stick_x * (1 - 0.66 * gamepad1.right_trigger);

                    } else {
                        x = -gamepad1.left_stick_x;
                        y = -gamepad1.left_stick_y;
                        rx = gamepad1.right_stick_x;
                    }
            robot.setDrivePower(-x, y, rx);
                    break;
                case ALIGN_TO_POINT:
                    Pose2d poseEstimate = robot.drive.getLocalizer().getPoseEstimate();
                    robot.relocalization.aprilTags.detectBackdrop();
                    AprilTagPoseFtc rawDifference = robot.relocalization.aprilTags.getRelativePose();

                    // Switch back into normal driver control mode if trigger is pressed
                    if (gamepad1.square && !previousAutoAlignState) {
                        currentMode = Mode.NORMAL_CONTROL;
                        robot.drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
                    }
                    previousAutoAlignState = gamepad1.square;

                    // ATTEMPTS TO TURN WILL CANCEL AUTO ALIGH
                    if (gamepad1.right_stick_x != 0) {
                        currentMode = Mode.NORMAL_CONTROL;
                        robot.drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
                    }

                    // Create a vector from the gamepad x/y inputs which is the field relative movement
                    // Then, rotate that vector by the inverse of that heading for field centric control
                    Vector2d fieldFrameInput = new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    );
                    Vector2d robotFrameInput = fieldFrameInput.rotated(-poseEstimate.getHeading());

                    Vector2d difference = new Vector2d(rawDifference.x, rawDifference.y);
                    double theta = difference.angle();

                    // Not technically omega because its power. This is the derivative of atan2
                    double thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm());

                    // Set the target heading for the heading controller to our desired angle
                    headingController.setTargetPosition(theta);

                    // Set desired angular velocity to the heading controller output + angular
                    // velocity feedforward
                    double headingInput = (headingController.update(poseEstimate.getHeading())
                            * DriveConstants.kV + thetaFF)
                            * DriveConstants.TRACK_WIDTH;

                    // Combine the field centric x/y velocity with our derived angular velocity
                    Pose2d driveDirection = new Pose2d(
                            robotFrameInput,
                            headingInput
                    );

                    robot.drive.setWeightedDrivePower(driveDirection);
                    headingController.update(poseEstimate.getHeading());
//                    if (headingInput <= 0.15 && headingInput >= -0.15) {
//                        //assume its aligned and switch back to hunty controls
//                        currentMode = Mode.NORMAL_CONTROL;
//                        gamepad1.rumbleBlips(2);
//                        robot.drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
//                    }
                    break;
            }


            //ARM
            if (gamepad2.left_trigger > 0.1) {
                robot.arm.setAngleArm((int) (robot.arm.arm1.getAngle() + (0.1*gamepad2.left_trigger)));
            } else if (gamepad2.right_trigger > 0.1) {
                robot.arm.setAngleArm((int) (robot.arm.arm1.getAngle() - (0.1*gamepad2.right_trigger)));
            }

            //CLAW
            if (gamepad1.cross) {
                robot.smartClawOpen();
            }

            if (gamepad1.square && !previousSquare) {
                robot.claw.wrist.setAngle(78);
            }
            if (gamepad1.circle && !previousCircle) {
                robot.claw.wrist.setAngle(168);
            }
            previousCircle = gamepad1.circle;
            previousSquare = gamepad1.square;


            //INTAKE
            if (gamepad1.right_bumper && (gamepad1.right_bumper != previousIntakeState)){
                if (robot.intaking) {
                    robot.stopIntake();
                } else {
                    robot.startIntake();
                }
            }
            previousIntakeState = gamepad1.right_bumper;

            if ((gamepad1.left_trigger > 0.2)){
                robot.intake.reverse();
            } else if ((gamepad1.left_trigger<0.2)  && (gamepad1.left_trigger != previousLeftTriggerState)){
                robot.intake.stopIntake();
            }
            previousLeftTriggerState = gamepad1.left_trigger;

            //DEPOSIT
            if (gamepad1.left_bumper) {
                robot.depositPreset();
            }

            //SLIDES
            if (gamepad1.dpad_left && !previousDpadLeftState) {
                robot.slides.incrementBackdropTarget(-70);
            } else if (gamepad1.dpad_right && !previousDpadRightState) {
                robot.slides.incrementBackdropTarget(70);
            }
            previousDpadLeftState = gamepad1.dpad_left;
            previousDpadRightState = gamepad1.dpad_right;

            //WRIST
            if (gamepad2.right_stick_x > 0.2) {
                robot.arm.setAngleElbow(robot.arm.elbow.getAngle() + 70);
            } else if (gamepad2.right_stick_x < -0.2) {
                robot.arm.setAngleElbow(robot.arm.elbow.getAngle() - 70);
            }

            //DRONE
            if (gamepad1.triangle && !previousDroneState) {
                robot.drone.launch();
            }
            previousDroneState = gamepad1.triangle;

//            // AUTO ALIGN
//            if (gamepad1.square && !previousAutoAlignState) {
//                currentMode = Mode.ALIGN_TO_POINT;
//                gamepad1.rumble(1);
//            }
//            previousAutoAlignState = gamepad1.square;

            // CLIMB
            if (gamepad1.dpad_up) {
                robot.climbExtend();
            }

            if (gamepad1.dpad_down) {
                robot.startClimb();
            }

            //TIME ALERTS
            if (buzzers == 0 && matchTimer.time(TimeUnit.SECONDS) >= 75) {
                gamepad1.rumble(500);
                gamepad2.rumble(500);
                buzzers = 1;
            } else if (buzzers == 1 && matchTimer.time(TimeUnit.SECONDS) >= 90) {
                gamepad1.rumble(800);
                gamepad2.rumble(800);
                buzzers = 2;
            }

            //autoClosePreviousState = gamepad1.circle;
            robot.slides.update();
            robot.antiJam();
            //robot.smartIntakeUpdate();
            //robot.drive.getLocalizer().update();
            telemetry.addData("TIME LEFT: ", ((120-matchTimer.time(TimeUnit.SECONDS))));
            telemetry.addData("CLAW POSITION: ", (robot.claw.depositServo.getAngle()));
            //telemetry.addData("ARM TARGET: ", (robot.arm.v4b1.servo.getPosition()*180));
            telemetry.addData("ARM POSITION: ", robot.arm.arm1.getAngle());
            telemetry.addData("SLIDES TARGET: ", robot.slides.target);
            telemetry.addData("SLIDES POSITION: ", robot.slides.slides1.motor.getCurrentPosition());
            telemetry.addData("LEVEL: ", robot.slides.currentLevel);
            telemetry.addData("DRIVE CURRENT ", robot.drive.getCurrent());
            telemetry.update();

//            PhotonCore.CONTROL_HUB.clearBulkCache();
//            PhotonCore.EXPANSION_HUB.clearBulkCache();

        }
    }
}