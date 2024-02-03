package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.AllianceColor;

import java.util.concurrent.TimeUnit;

public class IntakeSensor {
    ColorSensor sensor1;
    ColorSensor sensor2;
    ElapsedTime lastRead = new ElapsedTime();

    boolean[] detectedIndex = {false, false};

    DistanceUnit unit = DistanceUnit.MM;

    public IntakeSensor(ColorSensor sensor1, ColorSensor sensor2) {
        this.sensor1 = sensor1;
        this.sensor2 = sensor2;
    }

    public boolean[] hasPixel(){
        if (lastRead.time(TimeUnit.MILLISECONDS) <= 100) {
            return detectedIndex;
        }

        double sensorValue1 = getRangeSensor1();
        double sensorValue2 = getRangeSensor2();
        lastRead.reset();

        clearDetectedIndex();

        if (sensorValue1 >= 2.0 && sensorValue1 <= 11.0) {
            detectedIndex[0] = true;
        }

        if (sensorValue2 >= 2.0 && sensorValue2 <= 11.0) {
            detectedIndex[1] = true;
        }

        return detectedIndex;

    }

    public void clearDetectedIndex() {
        detectedIndex[0] = false;
        detectedIndex[1] = false;
    }

    public double getRangeSensor1() {
        return ((DistanceSensor) sensor1).getDistance(unit);
    }

    public double getRangeSensor2() {
        return ((DistanceSensor) sensor2).getDistance(unit);
    }
}