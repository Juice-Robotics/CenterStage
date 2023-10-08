package org.firstinspires.ftc.teamcode.subsystems.launcher;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.StepperServo;

public class DroneLauncher {
    Servo launcher;

    public DroneLauncher(StepperServo l) {
        launcher = l.servo;
    }

    public void prime() {
        launcher.setPosition(0);
    }

    public void launch() {
        launcher.setPosition(1);
    }
}
