package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public enum PixelColors {
    PURPLE(183f, 115f, 223f, 0.8f),
    YELLOW(253f, 239f, 130f, 0.8f),
    GREEN(83f, 234f, 40f, 0.8f),
    WHITE(239f, 239f, 239f, 0.8f);

    private final float red;
    private final float green;
    private final float blue;
    private final float alpha;

    PixelColors(float red, float green, float blue, float alpha) {
        this.red = red;
        this.green = green;
        this.blue = blue;
        this.alpha = alpha;
    }

    public static PixelColors getColor(NormalizedRGBA color, float epsilon) {
        for (PixelColors pcolor : PixelColors.values()) {
            if (Math.abs(color.red - pcolor.red) <= epsilon && Math.abs(color.green - pcolor.green) <= epsilon && Math.abs(color.blue - pcolor.blue) <= epsilon) {
                return pcolor;
            }
        }
        return null;
    }

}
