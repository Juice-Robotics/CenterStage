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

    public float OFFSET = 25;

    public MotorEx intakeMotor;

    public Intake(StepperServo intakeServo1, StepperServo intakeServo2, MotorEx intakeMotor) {
        this.intakeServo1 = intakeServo1;
        this.intakeServo2 = intakeServo2;

        this.intakeMotor = intakeMotor;
        intakeServo2.servo.setDirection(Servo.Direction.REVERSE);
        // intakeMotor.motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void startIntake(){
        intakeMotor.setSpeed(1);
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
        if (level == Levels.ZERO) {
            setAngle(0);
        } else if (level == Levels.INTAKE) {
            setAngle(220-OFFSET);
        } else if (level == Levels.INTERMEDIATE) {
            setAngle(130-OFFSET);
        } else if (level == Levels.CLIMB_EXTEND) {
            setAngle(130-OFFSET);
        }
    }

    public void reverse(){
        intakeMotor.setSpeed(-0.6F);
    }

}
