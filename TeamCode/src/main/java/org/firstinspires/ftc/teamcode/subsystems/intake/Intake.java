package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.Motor;
import org.firstinspires.ftc.teamcode.lib.StepperServo;

public class Intake {

    // Define class members
    double DOWNPOSITION = 0;
    Servo   intakeServo;
    DcMotor intakeMotor;
    double intakeSpeed = 0.8;
    double currentPos;

    public Intake(StepperServo intakeServo, Motor intakeMotor) {
        this.intakeServo = intakeServo.servo;
        this.intakeMotor = intakeMotor.motor;
        this.currentPos = intakeServo.servo.getPosition();
    }

    public void setIntakeSpeed(double intakeSpeed){
        this.intakeSpeed = intakeSpeed;
    }

    public void startIntake(){
        intakeMotor.setPower(intakeSpeed);
        intakeServo.setPosition(DOWNPOSITION);
    }
    public void stopIntake(){
        intakeServo.setPosition(DOWNPOSITION+0.5);
        intakeMotor.setPower(0);
    }
    public void goToPos(double Position){
        intakeServo.setPosition(Position);
    }
    public void startMotor(){
        intakeMotor.setPower(intakeSpeed);
    }



}
