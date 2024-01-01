package org.firstinspires.ftc.teamcode.subsystems.deposit;

import org.firstinspires.ftc.teamcode.lib.Levels;
import org.firstinspires.ftc.teamcode.lib.StepperServo;

public class Claw {
    public StepperServo depositServo;
    public StepperServo wrist;

    public boolean isOpen = false; // if open, true

    // CONSTANTS
    public float clawOpen = 200;
    public float clawClose = 245; //larger = tighter

    public Claw(StepperServo depositServo, StepperServo wrist) {
        this.depositServo = depositServo;
        this.wrist = wrist;
        // this.wrist.servo.setDirection(Servo.Direction.REVERSE);
    }

    public void toggle() {
        // Close Claw
        if (this.isOpen) {
            this.depositServo.servo.setPosition(clawClose);
            this.isOpen = false;
        }
        // Open Claw
        else {
            this.depositServo.servo.setPosition(clawOpen);
            this.isOpen = true;
        }
    }

    public void setPositionClaw(float angle) {
        this.depositServo.setAngle(angle);
    }

    public void setPositionWrist(float rotation) {
        this.wrist.setAngle(rotation);
    }

    public void runToWristPreset(Levels level) {
        if (level == Levels.ZERO) {
            setPositionWrist(0);
        } else if (level == Levels.INTAKE) {
            setPositionWrist(123);
        } else if (level == Levels.DEPOSIT) {
            setPositionWrist(123);
        }
    }

    public void setClawOpen() {
        this.depositServo.setAngle(clawOpen);
    }

    public void setClawClose() {
        this.depositServo.servo.setPosition(clawClose);
    }

}
