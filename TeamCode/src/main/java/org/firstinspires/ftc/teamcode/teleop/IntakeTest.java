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
@Config
public class IntakeTest extends LinearOpMode {
    DcMotorEx motor;
    public static double MOT_POWER = 0;
    public static double ARM_POS = 27;
    public static double ELBOW = 110;
    public static double WRIST_PIVOT = 123;
    public static double INTAKE_DEPLOY = 197;
    public static double CLAW = 185;



    StepperServo elbow;
    StepperServo wrist2;

    StepperServo arm1;
    StepperServo arm2;

    StepperServo intakeDeploy1;
    StepperServo intakeDeploy2;
    StepperServo claw;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        elbow = new StepperServo(0, "elbow", hardwareMap);
        wrist2 = new StepperServo(0, "wrist", hardwareMap);
        arm1 = new StepperServo(0, "arm1", hardwareMap);
        arm2 = new StepperServo(0, "arm2", hardwareMap);
        intakeDeploy1 = new StepperServo(0, "intakeServo1", hardwareMap);
        intakeDeploy2 = new StepperServo(0, "intakeServo2", hardwareMap);
        claw = new StepperServo(0, "claw", hardwareMap);
        intakeDeploy2.servo.setDirection(Servo.Direction.REVERSE);

        // Initialize your own robot class
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            motor.setPower(MOT_POWER);
            elbow.setAngle((float) ELBOW);
            wrist2.setAngle((float) WRIST_PIVOT);
            arm1.setAngle((float) ARM_POS);
            arm2.setAngle((float) ARM_POS);
            intakeDeploy1.setAngle((float) INTAKE_DEPLOY);
            intakeDeploy2.setAngle((float) INTAKE_DEPLOY);
            claw.setAngle((float) CLAW);

            telemetry.addData("current draw", motor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("velocity", motor.getVelocity());
            telemetry.update();
        }
    }
}


// twist wrist 180
// pivot wrist 90
// axons: 120
//intake 70%