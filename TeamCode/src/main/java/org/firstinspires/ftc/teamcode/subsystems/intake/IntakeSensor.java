package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeSensor{
    NormalizedColorSensor colorSensor;
    NormalizedColorSensor colorSensor2;
    DistanceSensor distanceSensor1;
    DistanceSensor distanceSensor2;

    NormalizedRGBA colors;
    NormalizedRGBA colors2;
    NormalizedRGBA indexColor;
    NormalizedRGBA[] currentState;
    float gain = 2;

    /**
    <h2>Unit: MM</h2>
     */
    double distance1;
    /**
     <h2>Unit: MM</h2>
     */
    double distance2;
    final double distance1Tol = 0.5;
    final double distance2Tol = 0.5;

    boolean[] detectedIndex;

    public IntakeSensor(NormalizedColorSensor colorSensor, NormalizedColorSensor colorSensor2, float gain) {
        this.colorSensor = colorSensor;
        this.colorSensor2 = colorSensor2;
        this.gain = gain;
    }

    public IntakeSensor(DistanceSensor ds1, DistanceSensor ds2) {
        this.distanceSensor1 = ds1;
        this.distanceSensor2 = ds2;
    }

    public NormalizedRGBA[] getState() {
        colors = colorSensor.getNormalizedColors();
        colors2 = colorSensor2.getNormalizedColors();
        currentState[0] = colors;
        currentState[1] = colors2;
        return currentState;
    }

    public boolean[] hasPixel(){
        colors = colorSensor.getNormalizedColors();
        colors2 = colorSensor2.getNormalizedColors();

        clearDetectedIndex();
        if (colors != indexColor) {
            detectedIndex[0] = true;
        }
        if (colors2 != indexColor) {
            detectedIndex[1] = true;
        }
        return detectedIndex;
    }

    public boolean[] hasPixelDist() {
        distance1 = distanceSensor1.getDistance(DistanceUnit.MM);
        distance2 = distanceSensor2.getDistance(DistanceUnit.MM);

        if (distance1 <= distance1Tol) {
            detectedIndex[0] = true;
        }
        if (distance2 <= distance2Tol) {
            detectedIndex[1] = true;
        }

        return detectedIndex;
    }

    public boolean[] cachedHasPixel() {
        return detectedIndex;
    }

    public void clearDetectedIndex() {
        detectedIndex[0] = false;
        detectedIndex[1] = false;
    }


}



