package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group = "competition")
@Config
public class MaxVerOdoen extends LinearOpMode {
    DcMotor verstappen = hardwareMap.get(DcMotor.class, "slides1");
    public static double SPEED = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize your own robot class
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            verstappen.setPower(SPEED);
        }
    }
}