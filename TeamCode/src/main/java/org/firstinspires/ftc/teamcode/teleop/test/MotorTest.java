package org.firstinspires.ftc.teamcode.teleop.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.AllianceColor;
import org.firstinspires.ftc.teamcode.lib.StepperServo;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Claw;

@TeleOp(group = "competition")
@Config
@Disabled
public class MotorTest extends LinearOpMode {
    DcMotorEx motor;
    public static double MOT_POWER = 1.0;

    public double currentWindowTotal = 0;
    public int currentWindowLength = 0;

    public double velocityWindowTotal = 0;
    public int velocityWindowLength = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motor = hardwareMap.get(DcMotorEx.class, "leftFront");
        // Initialize your own robot class
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            motor.setPower(MOT_POWER);

            currentWindowTotal += motor.getCurrent(CurrentUnit.AMPS);
            currentWindowLength += 1;
            velocityWindowTotal += motor.getVelocity(AngleUnit.DEGREES);
            velocityWindowLength += 1;

            telemetry.addData("current draw", motor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("velocity", motor.getVelocity(AngleUnit.DEGREES));
            telemetry.addData("average current draw", currentWindowTotal / currentWindowLength);
            telemetry.addData("average velocity", velocityWindowTotal / velocityWindowLength);
            telemetry.update();
        }
    }
}