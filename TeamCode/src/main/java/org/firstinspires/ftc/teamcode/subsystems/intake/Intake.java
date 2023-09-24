package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    // Define class members
    double DOWNPOSITION = 0;
    Servo   intakeServo;
    DcMotor intakeMotor;
    double intakeSpeed = 0.8;
    double currentPos;

    public Intake(Servo intakeServo, DcMotor intakeMotor) {
        this.intakeServo = intakeServo;
        this.intakeMotor = intakeMotor;
        this.currentPos = intakeServo.getPosition();
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