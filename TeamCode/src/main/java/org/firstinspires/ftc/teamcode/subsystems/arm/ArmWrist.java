package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.StepperServo;

public class ArmWrist {
    public Servo arm1;
    public Servo arm2;
    public Servo wrist;

    public double currentAngle;

    //presets
    public double[] preset1 = {1,1}; //arm, wrist

    public ArmWrist(StepperServo arm1, StepperServo arm2, StepperServo wrist) {
        this.arm1 = arm1.servo;
        this.arm2 = arm2.servo;
        this.wrist = wrist.servo;
        arm1.servo.setDirection(Servo.Direction.REVERSE);
        currentAngle = arm1.servo.getPosition();
    }
    public void turnToPreset(Levels level){
        if (level == Levels.ZERO) {
            arm1.setPosition(0);
            arm2.setPosition(0);
            wrist.setPosition(0);
        } else if (level == Levels.INTAKE) {
            arm1.setPosition(0);
            arm2.setPosition(0);
            wrist.setPosition(0);
        }
    }

    public void wristToPos(double wristPos){
        wrist.setPosition(wristPos);
    }
    public void armToPos(double armPos){
        arm1.setPosition(armPos);
        arm2.setPosition(armPos);
    }



}
