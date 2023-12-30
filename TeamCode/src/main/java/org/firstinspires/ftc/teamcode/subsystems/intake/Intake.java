package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.Motor;
import org.firstinspires.ftc.teamcode.lib.MotorEx;
import org.firstinspires.ftc.teamcode.lib.StepperServo;

public class Intake {

    public StepperServo intakeServo1;
    public StepperServo intakeServo2;

    public float intakeUp = 100;

    public float intakeDown = 25;

    public MotorEx intakeMotor;

    public Intake(StepperServo intakeServo1, StepperServo intakeServo2, MotorEx intakeMotor) {
        this.intakeServo1 = intakeServo1;
        this.intakeServo2 = intakeServo2;

        this.intakeMotor = intakeMotor;
        intakeServo2.servo.setDirection(Servo.Direction.REVERSE);
        // intakeMotor.motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void startIntake(){
        intakeMotor.setSpeed(0.6F);
        intakeServo1.setAngle(intakeDown);
    }

    public void stopIntake(){
        intakeMotor.setSpeed(0);
        intakeServo1.setAngle(intakeUp);
    }

    public void setAngle(float angle) {
        intakeServo1.setAngle(angle);
        intakeServo2.setAngle(angle);
    }

    public void runToPreset(Levels level) {
        switch (level) {
            case ZERO:
                setAngle(0);
                break;
            case INTAKE:
                setAngle(192);
                break;
            case INTERMEDIATE:
                setAngle(130);
                break;
            case CLIMB_EXTEND:
                setAngle(130);
        }
    }

    public void reverse(){
        intakeMotor.setSpeed(-0.6F);
    }

}
