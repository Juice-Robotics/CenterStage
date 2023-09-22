package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.qualcomm.robotcore.hardware.Servo;

public class Deposit {
    public Servo depositServo;

    public double currentAngle;

    //presets (????????????????????)
    public double open = 1;
    public double close = 0;
    public double epsilon = 0.1;

    public Deposit(Servo depositServo) {
        this.depositServo = depositServo;
        currentAngle = depositServo.getPosition();
    }
    public void open(){
        depositServo.setPosition(open);
        currentAngle = open;
    }
    public void close(){
        depositServo.setPosition(close);
        currentAngle = close;
    }
    public void toggle(){
        if (currentAngle == open){
            close();
        }
        else{
            open();
        }
    }
    public void depositToPos(double pos){
        depositServo.setPosition(pos);
    }

}
