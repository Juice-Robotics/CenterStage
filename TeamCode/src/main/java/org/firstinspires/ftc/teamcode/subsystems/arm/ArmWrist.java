package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.StepperServo;

public class ArmWrist {
    public Servo arm1;
    public Servo arm2;
    public Servo wrist;

    public double currentAngle;

    private MotionProfile profile;
    public MotionState curState;
    private ElapsedTime timer;
    double maxvel = 6000;
    double maxaccel = 6000;

    //presets
    public double[] preset1 = {1,1}; //arm, wrist

    public ArmWrist(StepperServo arm1, StepperServo arm2, StepperServo wrist) {
        this.arm1 = arm1.servo;
        this.arm2 = arm2.servo;
        this.wrist = wrist.servo;
        arm1.servo.setDirection(Servo.Direction.REVERSE);
        currentAngle = arm1.servo.getPosition();

        timer = new ElapsedTime();
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(1, 0), new MotionState(0, 0), maxvel, maxaccel);
    }

    public void turnToPreset(Levels level){ //testtesttesttesttest
        if (level == Levels.ZERO) {
            armToPos(0);
            wrist.setPosition(0);
        } else if (level == Levels.INTAKE) {
            armToPos(0);
            wrist.setPosition(0);
        } else if (level == Levels.DEPOSIT){
            armToPos(0);
            wrist.setPosition(0);
        }
    }

    public void wristToPos(double wristPos){
        wrist.setPosition(wristPos);
    }
    public void armToPos(double armPos){
        profile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(arm1.getPosition(), 0), new MotionState(armPos, 0), maxvel, maxaccel);
        timer.reset();
    }

    public void update() {
        double armPos = profile.get(timer.time()).getX();
        arm1.setPosition(armPos);
        arm2.setPosition(armPos);
    }
}
