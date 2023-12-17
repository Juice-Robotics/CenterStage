package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.lib.AllianceColor;
import org.firstinspires.ftc.teamcode.lib.StepperServo;
import org.firstinspires.ftc.teamcode.subsystems.deposit.Claw;

@TeleOp(group = "competition")
@Config
@Disabled
public class ServoZero extends LinearOpMode {
    Servo servo;
    Servo servo2;
    public static double DEP_POS = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "arm1");
        servo2 = hardwareMap.get(Servo.class, "arm2");
        // Initialize your own robot class
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            servo.setPosition(DEP_POS);
        }
    }
}