package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class IntakeSensor{
    NormalizedColorSensor colorSensor;
    NormalizedColorSensor colorSensor2;
    NormalizedRGBA colors;
    NormalizedRGBA colors2;
    NormalizedRGBA indexColor;
    NormalizedRGBA[] currentState;
    boolean[] currentBoolean;
    float gain = 2;

    public IntakeSensor(NormalizedColorSensor colorSensor, NormalizedColorSensor colorSensor2, float gain) {
        this.colorSensor = colorSensor;
        this.colorSensor2 = colorSensor2;
        this.gain = gain;
    }
    public NormalizedRGBA[] getState(){
        colors = colorSensor.getNormalizedColors();
        colors2 = colorSensor2.getNormalizedColors();
        currentState[0] = colors;
        currentState[1] = colors2;
        return currentState;
    }
    public boolean[] hasPixel(){
        colors = colorSensor.getNormalizedColors();
        colors2 = colorSensor2.getNormalizedColors();
        if (colors != indexColor){
            currentBoolean[0] = true;
        }
        if(colors2 != indexColor){
            currentBoolean[1] = true;
        }
        return currentBoolean;
    }


}



