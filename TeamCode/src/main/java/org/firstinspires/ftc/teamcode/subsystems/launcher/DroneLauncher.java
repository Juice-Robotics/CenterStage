package org.firstinspires.ftc.teamcode.subsystems.launcher;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.lib.StepperServo;

import org.firstinspires.ftc.teamcode.lib.StepperServo;

public class DroneLauncher {
    StepperServo drone;

    public DroneLauncher(StepperServo drone) {
        this.drone = drone;
    }

    public void prime() {
        drone.setAngle(0);
    }

    public void launch() {
        drone.setAngle(100);
    }
}
