package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.qualcomm.robotcore.hardware.Servo;

public class ArmWrist {
    public Servo arm1;
    public Servo arm2;
    public Servo wrist;

    public double currentAngle;

    //presets
    public double[] preset1 = {1,1}; //arm, wrist

    public ArmWrist(Servo arm1, Servo arm2, Servo wrist) {
        this.arm1 = arm1;
        this.arm2 = arm2;
        this.wrist = wrist;
        arm1.setDirection(Servo.Direction.REVERSE);
        currentAngle = arm1.getPosition();
    }
    public void turnToPreset(double [] preset){
        arm1.setPosition(preset[0]);
        arm2.setPosition(preset[0]);
        wrist.setPosition(preset[1]);
    }

    public void wristToPos(double wristPos){
        wrist.setPosition(wristPos);
    }
    public void armToPos(double armPos){
        arm1.setPosition(armPos);
        arm2.setPosition(armPos);
    }



}
