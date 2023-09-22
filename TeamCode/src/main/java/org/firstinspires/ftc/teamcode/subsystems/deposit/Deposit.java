package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.qualcomm.robotcore.hardware.Servo;

public class Deposit {
    public Servo depositServo;

    public double currentAngle;

    //presets (????????????????????)
    public double open = 1;
    public double close = 0;

    public Deposit(Servo depositServo) {
        this.depositServo = depositServo;
        currentAngle = depositServo.getPosition();
    }
    public void turnToPreset(double preset){
        depositServo.setPosition(preset);
    }

    public void depositToPos(double pos){
        depositServo.setPosition(pos);
    }

}
