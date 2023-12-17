package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.lib.StepperServo;

@TeleOp(group = "competition")
@Config
@Disabled
public class ServoTest extends LinearOpMode {
    public static double TWO_POS = 0;
    public static double ONE_POS = 0;


    StepperServo one;
    StepperServo two;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        one = new StepperServo(0, "arm1", hardwareMap);
        two = new StepperServo(0, "arm2", hardwareMap);

        // Initialize your own robot class
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            one.setAngle((float) ONE_POS);
            two.setAngle((float) ONE_POS);
        }
    }
}


// twist wrist 180
// pivot wrist 90
// axons: 120
//intake 70%