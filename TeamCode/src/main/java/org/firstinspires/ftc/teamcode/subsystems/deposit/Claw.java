package org.firstinspires.ftc.teamcode.subsystems.deposit;

import org.firstinspires.ftc.teamcode.lib.StepperServo;

public class Claw {
    public StepperServo depositServo;
    public StepperServo wrist;

    public boolean isOpen = false; // if open, true

    // CONSTANTS
    public double clawOpen = 0.65;
    public double clawClose = 0.25; //smaller = tighter

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

    public void setPositionClaw(double angle) {
        this.depositServo.servo.setPosition(angle);
    }

    public void setPositionWrist(float rotation) {
        this.wrist.setAngle(rotation);
    }

    public void setClawOpen() {
        this.depositServo.servo.setPosition(clawOpen);
    }

    public void setClawClose() {
        this.depositServo.servo.setPosition(clawClose);
    }

}