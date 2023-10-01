package org.firstinspires.ftc.teamcode.subsystems.deposit;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.StepperServo;

public class Deposit {
    public Servo depositServo;

    public double currentAngle;

    //presets (????????????????????)
    public double open = 0.5;
    public double close = 1;

    public Deposit(StepperServo depositServo) {
        this.depositServo = depositServo.servo;
        currentAngle = depositServo.servo.getPosition();
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
    public void setPos(double pos){
        depositServo.setPosition(pos);
    }

}
