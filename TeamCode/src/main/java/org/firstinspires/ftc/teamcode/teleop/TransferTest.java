package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.StepperServo;

@TeleOp(group = "competition")
@Config
public class TransferTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Robot robot = new Robot(hardwareMap, false);

        // Initialize your own robot class
        waitForStart();
        if (isStopRequested()) return;
        robot.startIntake();
        robot.slides.resetAllEncoders();
        robot.arm.setAngleArm(6);
        robot.claw.setPositionClaw(140);
        robot.intake.setAngle(192);
        robot.claw.wrist.setAngle(123);
        robot.arm.setAngleElbow(112);
        robot.slides.runToPosition(0);
        boolean previousX = gamepad1.cross;
        boolean previousBumper = gamepad1.left_bumper;
        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad1.cross && !previousX) {
                robot.arm.setAngleArm(0);
                try {
                    Thread.sleep(250);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                robot.claw.setPositionClaw(215);
                robot.intake.stopIntake();
                robot.intake.setAngle(130);
                try {
                    Thread.sleep(250);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                robot.arm.setAngleArm(15);
                robot.arm.setAngleElbow(115);
//                ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//                double start = timer.time();
//                while (timer.time() - start <= 200) {
//                    robot.slides.update();
//                }
//                robot.slides.runToPosition(100);
//                try {'mm
//                    Thread.sleep(75);
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                }
//                robot.arm.setAngleArm(135);
//                try {
//                    Thread.sleep(75);
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                }
//                robot.slides.runToPosition(0);
            }

            if (gamepad1.left_bumper && !previousBumper) {
                robot.slides.runToPosition(400);
                ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                double start = timer.time();
                while (timer.time() - start <= 600) {
                    robot.slides.update();
                }
                robot.arm.setAngleArm(150);
                robot.arm.setAngleElbow(180);
            }

            previousX = gamepad1.cross;
            previousBumper = gamepad2.left_bumper;
            robot.slides.update();
        }
    }
}


// twist wrist 180
// pivot wrist 90
// axons: 120
//intake 70%