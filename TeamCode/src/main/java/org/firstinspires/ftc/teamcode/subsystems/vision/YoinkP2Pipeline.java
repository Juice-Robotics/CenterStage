package org.firstinspires.ftc.teamcode.subsystems.vision;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.fasterxml.jackson.databind.annotation.JsonAppend;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.lib.AllianceColor;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.atomic.AtomicReference;
@Config
public class YoinkP2Pipeline implements VisionProcessor, CameraStreamSource {
    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
    private Mat testMat = new Mat();
    private Mat highMat = new Mat();
    private Mat lowMat = new Mat();
    private Mat finalMat = new Mat();

    public AllianceColor alliance = AllianceColor.BLUE;
    public static double redThreshold = 0.11;
    public static double blueThreshold = 0.1;
    double threshold = 0;
    private PropLocation location = PropLocation.LEFT;

    private final Scalar lowHSVRedLower = new Scalar(0, 100, 20);  //Beginning of Color Wheel
    private final Scalar lowHSVRedUpper = new Scalar(10, 255, 255);

    private final Scalar highHSVRedLower = new Scalar(160, 100, 20); //Wraps around Color Wheel
    private final Scalar highHSVRedUpper = new Scalar(180, 255, 255);
    private final Scalar lowHSVBlueLower = new Scalar(80, 100, 100);
    private final Scalar highHSVBlueUpper = new Scalar(170, 255, 255);
    static final Rect LEFT_RECTANGLE = new Rect(0, 151, 180, 179);
    static final Rect CENTER_RECTANGLE = new Rect(200, 120, 340, 240);
    Telemetry telemetry;

    public YoinkP2Pipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        if (alliance == AllianceColor.RED) {
            // RED
            threshold = redThreshold;
        }
        else {
            // BLUE
            threshold = blueThreshold;
        }
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Convert Image from RGB to HSV
        Imgproc.cvtColor(frame, testMat, Imgproc.COLOR_RGB2HSV);

        if (alliance == AllianceColor.RED) {
            Core.inRange(testMat, lowHSVRedLower, lowHSVRedUpper, lowMat);
            Core.inRange(testMat, highHSVRedLower, highHSVRedUpper, highMat);
            Core.bitwise_or(lowMat, highMat, finalMat);
        } else {
            Core.inRange(testMat, lowHSVBlueLower, highHSVBlueUpper, finalMat);
        }

        testMat.release();
        lowMat.release();
        highMat.release();

        double leftBox = Core.sumElems(finalMat.submat(LEFT_RECTANGLE)).val[0];
        double rightBox = Core.sumElems(finalMat.submat(CENTER_RECTANGLE)).val[0];

        double averagedLeftBox = leftBox / LEFT_RECTANGLE.area() / 255;
        double averagedRightBox = rightBox / CENTER_RECTANGLE.area() / 255; //Makes value [0,1]
        telemetry.addData("Left Box: ", averagedLeftBox);
        telemetry.addData("Right Box: ", averagedRightBox);
        telemetry.update();
        if (averagedLeftBox > threshold) {        //Must Tune Threshold
            location = PropLocation.LEFT;
            Imgproc.rectangle(finalMat, LEFT_RECTANGLE, new Scalar(0, 255, 0));
        } else if (averagedRightBox > threshold) {
            location = PropLocation.CENTER;
            Imgproc.rectangle(finalMat, CENTER_RECTANGLE, new Scalar(0, 255, 0));
        } else {
            location = PropLocation.RIGHT;
            Imgproc.rectangle(finalMat, CENTER_RECTANGLE, new Scalar(75, 0, 130));
        }

        // These lines are for tuning the rectangles
        Imgproc.rectangle(finalMat, LEFT_RECTANGLE, new Scalar(255, 255, 255));
        Imgproc.rectangle(finalMat, CENTER_RECTANGLE, new Scalar(255, 255, 255));

        finalMat.copyTo(frame); /*This line should only be added in when you want to see your custom pipeline
                                  on the driver station stream, do not use this permanently in your code as
                                  you use the "frame" mat for all of your pipelines, such as April Tags*/
        Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, b);
        lastFrame.set(b);
        return null;            //You do not return the original mat anymore, instead return null

    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }


    public PropLocation getPropPosition() {
        return this.location;
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    public enum PropLocation {
        LEFT,
        CENTER,
        RIGHT,
        UNFOUND
    }
}