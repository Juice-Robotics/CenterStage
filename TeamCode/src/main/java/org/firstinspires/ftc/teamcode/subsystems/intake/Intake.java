package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.Motor;
import org.firstinspires.ftc.teamcode.lib.StepperServo;

public class Intake {

    public StepperServo intakeServo1;
    public StepperServo intakeServo2;

    public float intakeUp = 100;

    public float intakeDown = 25;

    public DcMotorEx intakeMotor;

    public Intake(StepperServo intakeServo1, StepperServo intakeServo2, DcMotorEx intakeMotor) {
        this.intakeServo1 = intakeServo1;
        this.intakeServo2 = intakeServo2;

        this.intakeMotor = intakeMotor;
        intakeServo2.servo.setDirection(Servo.Direction.REVERSE);
        // intakeMotor.motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void startIntake(){
        intakeMotor.setPower(1);
        intakeServo1.setAngle(intakeDown);
    }

    public void stopIntake(){
        intakeMotor.setPower(0);
        intakeServo1.setAngle(intakeUp);
    }

    public void setAngle(float angle) {
        intakeServo1.setAngle(angle);
        intakeServo2.setAngle(angle);
    }
    public void reverse(){
        intakeMotor.setPower(-1);
    }

}
