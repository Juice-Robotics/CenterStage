package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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
        robot.arm.setAngleArm(6);
        robot.claw.setPositionClaw(140);
        robot.intake.setAngle(192);
        robot.claw.wrist.setAngle(123);
        robot.arm.setAngleElbow(112);
        boolean previousX = gamepad1.cross;
        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad1.cross && !previousX) {
                robot.arm.setAngleArm(0);
                robot.claw.setPositionClaw(200);
                robot.intake.stopIntake();
                robot.intake.setAngle(120);
                try {
                    Thread.sleep(200);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                robot.arm.setAngleArm(15);
                robot.arm.setAngleElbow(115);

            }

            previousX = gamepad1.cross;
        }
    }
}


// twist wrist 180
// pivot wrist 90
// axons: 120
//intake 70%