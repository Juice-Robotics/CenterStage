package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.lib.StepperServo;

@TeleOp(group = "competition")
@Disabled
@Config
public class DroneTuning extends LinearOpMode {
    public static double DRONE = 0;



    StepperServo drone;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drone = new StepperServo(0, "drone", hardwareMap);


        // Initialize your own robot class
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            drone.setAngle((float) DRONE);
        }
    }
}


// twist wrist 180
// pivot wrist 90
// axons: 120
//intake 70%