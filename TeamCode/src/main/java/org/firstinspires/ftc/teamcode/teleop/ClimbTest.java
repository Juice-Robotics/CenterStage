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
public class ClimbTest extends LinearOpMode {
    DcMotorEx motor;
    public static double MOT_POWER = 0;
    public static double SHIFTER_POS = 83;

    StepperServo shifterServo;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motor = hardwareMap.get(DcMotorEx.class, "climb");
        shifterServo = new StepperServo(0, "shifter", hardwareMap);
        // shifterServo.servo.setDirection(Servo.Direction.REVERSE);

        // Initialize your own robot class
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            motor.setPower(MOT_POWER);
            shifterServo.setAngle((float) SHIFTER_POS);
        }
    }
}


// twist wrist 180
// pivot wrist 90
// axons: 120
//intake 70%