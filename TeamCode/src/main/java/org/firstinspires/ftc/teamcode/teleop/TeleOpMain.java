package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.AllianceColor;

import java.util.concurrent.TimeUnit;

@TeleOp(group = "competition")
public class TeleOpMain extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize your own robot class
        Robot robot = new Robot(hardwareMap,false);

        double x;
        double y;
        double rx;

        boolean previousClawSensorState = false;
        ElapsedTime clawSensorTimeout = new ElapsedTime();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

//        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
//        PhotonCore.experimental.setMaximumParallelCommands(4);
//        PhotonCore.enable();

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime matchTimer;

        int buzzers = 0;

        boolean autoCloseEnabled = true;
        boolean autoClosePreviousState = false;
        boolean previousClawState = false;
        boolean previousDroneState = false;
        int dronePressed = 0;


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;
        matchTimer = new ElapsedTime();

        while (opModeIsActive() && !isStopRequested()) {

//            if (gamepad1.dpad_up) {
//                robot.slides.resetAllEncoders();
//            }

            //DRIVE
            if (gamepad1.left_trigger > 0.5){
                x = -gamepad1.left_stick_x*(1-0.66*gamepad1.left_trigger);
                y = -gamepad1.left_stick_y*(1-0.66*gamepad1.left_trigger);
                rx = gamepad1.right_stick_x*(1-0.66*gamepad1.left_trigger);

            } else{
                x = -gamepad1.left_stick_x;
                y = -gamepad1.left_stick_y;
                rx = gamepad1.right_stick_x;
            }
            robot.setDrivePower(-x, y, rx);


            //ARM
            if (gamepad2.left_trigger > 0.1) {
                robot.arm.armToPos((int) (robot.arm.arm1.getPosition() + (0.1*gamepad2.left_trigger)));
            } else if (gamepad2.right_trigger > 0.1) {
                robot.arm.armToPos((int) (robot.arm.arm1.getPosition() - (0.1*gamepad2.right_trigger)));
            }

            //CLAW
            boolean isPressed = gamepad1.triangle;
            if (isPressed && !previousClawState) {
                robot.deposit.toggle();
            }
            previousClawState = isPressed;


            //INTAKE
            if (gamepad1.right_trigger > 0.5){
                robot.intakePreset();
            }
            if (gamepad1.right_bumper){
                robot.startSmartIntake();
            }

            //DEPOSIT
            if (gamepad1.left_bumper) {
                robot.depositPreset();
            }

            //SLIDES
            if (gamepad1.dpad_up) {
                robot.slides.runToPosition((int) (robot.slides.slides1.motor.getCurrentPosition() + 70));
            } else if (gamepad1.dpad_down) {
                robot.slides.runToPosition((int) (robot.slides.slides1.motor.getCurrentPosition() - 70));
            }

            //WRIST
            if (gamepad2.right_stick_x > 0.2) {
                robot.arm.wristToPos(robot.arm.wrist.getPosition() + 70);
            } else if (gamepad2.right_stick_x < -0.2) {
                robot.arm.wristToPos(robot.arm.wrist.getPosition() - 70);
            }

            //DRONE
            boolean isPressed2 = gamepad1.circle;
            if (gamepad1.square && !previousDroneState && (dronePressed==0)) {
                robot.drone.prime();
                dronePressed = 1;
            }
            else if (gamepad1.square && !previousDroneState && (dronePressed==1)) {
                robot.drone.launch();
                dronePressed = 2;
            }
            previousDroneState = isPressed2;

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
            telemetry.addData("TIME LEFT: ", ((120-matchTimer.time(TimeUnit.SECONDS))));
            telemetry.addData("CLAW POSITION: ", (robot.deposit.depositServo.getPosition()));
            //telemetry.addData("ARM TARGET: ", (robot.arm.v4b1.servo.getPosition()*180));
            telemetry.addData("ARM POSITION: ", robot.arm.arm1.getPosition());
            telemetry.addData("SLIDES TARGET: ", robot.slides.target);
            telemetry.addData("SLIDES POSITION: ", robot.slides.slides1.motor.getCurrentPosition());
            telemetry.addData("LEVEL: ", robot.slides.currentLevel);
            telemetry.update();

//            PhotonCore.CONTROL_HUB.clearBulkCache();
//            PhotonCore.EXPANSION_HUB.clearBulkCache();

        }
    }
}