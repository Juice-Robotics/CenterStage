package org.firstinspires.ftc.teamcode.subsystems.arm;

import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.StepperServo;

public class ArmElbow {
    public StepperServo arm1;
    public StepperServo arm2;
    public StepperServo elbow;

    public double currentAngle;

    // TARGETS
    public double intakeTargetArm = 27;
    public double depositTargetArm = 148;
    public double intakeTargetElbow = 110;
    public double depositTargetElbow = 217;

    public ArmElbow(StepperServo arm1, StepperServo arm2, StepperServo elbow) {
        this.arm1 = arm1;
        this.arm2 = arm2;
        this.elbow = elbow;
    }

    public void setAngleArm(double angle) {
        this.arm1.setAngle((float) angle);
        this.arm2.setAngle((float) angle);
        this.currentAngle = angle;
    }

    public void setAngleElbow(double angle) {
        this.elbow.setAngle((float) angle);
        this.currentAngle = angle;
    }

    public double getAngle() {
        return currentAngle;
    }

    public void runtoPreset(Levels level){
        if (level == Levels.INTAKE) {
            this.setAngleArm(intakeTargetArm);
            this.setAngleElbow(intakeTargetElbow);
        } else if (level == Levels.ZERO){
            this.setAngleArm(15);
            this.setAngleElbow(115);
        }
        else if (level == Levels.DEPOSIT) {
            this.setAngleArm(depositTargetArm);
            this.setAngleElbow(depositTargetElbow);
        }
        else if (level == Levels.INTERMEDIATE) {
            this.setAngleArm(30);
            this.setAngleElbow(115);
        }
        else if (level == Levels.CLIMB_EXTEND) {
            this.setAngleArm(160);
            this.setAngleElbow(237);
        }
        else if (level == Levels.CAPTURE) {
            this.setAngleArm(0);
            this.setAngleElbow(119);
        }
    }

}
