package org.firstinspires.ftc.teamcode.subsystems.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

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
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.atomic.AtomicReference;

@Config
public class YoinkElementCVProcessor implements VisionProcessor, CameraStreamSource {
    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    private PropLocation location = PropLocation.RIGHT;
    public MatOfKeyPoint keyPoints = new MatOfKeyPoint();

    private Telemetry telemetry;

    public AllianceColor alliance = AllianceColor.BLUE;
    private Rect leftZoneArea;
    private Rect centerZoneArea;

    private Mat finalMat = new Mat();

    public static int blueLeftX = 10;
    public static int blueLeftY = 240;

    public static int blueCenterX = 350;
    public static int blueCenterY = 240;

    public static int redLeftX = 10;
    public static int redLeftY = 240;

    public static int redCenterX = 371;
    public static int redCenterY = 240;

    public static int width = 125;
    public static int height = 125;


    public static double redThreshold = 4;
    public static double blueThreshold = 2;
    public static double threshold = 0;

    public double leftColor = 0.0;
    public double centerColor = 0.0;

    public Scalar left = new Scalar(0,0,0);
    public Scalar center = new Scalar(0,0,0);

    public YoinkElementCVProcessor(AllianceColor color, Telemetry telemetry) {
        this.alliance = color;
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));

        if (alliance == AllianceColor.RED) {
            threshold = redThreshold;
        } else {
            threshold = blueThreshold;
        }
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        if (alliance == AllianceColor.RED) {
            threshold = redThreshold;
        } else {
            threshold = blueThreshold;
        }

        frame.copyTo(finalMat);
        Imgproc.GaussianBlur(finalMat, finalMat, new Size(5, 5), 0.0);

        leftZoneArea = new Rect(alliance == AllianceColor.RED? redLeftX : blueLeftX, alliance == AllianceColor.RED? redLeftY : blueLeftY, width, height);
        centerZoneArea = new Rect(alliance == AllianceColor.RED?redCenterX:blueCenterX, alliance == AllianceColor.RED?redCenterY:blueCenterY, width, height);

        Mat leftZone = finalMat.submat(leftZoneArea);
        Mat centerZone = finalMat.submat(centerZoneArea);

        left = Core.sumElems(leftZone);
        center = Core.sumElems(centerZone);

        leftColor = left.val[0] / 333333.0;
        centerColor = center.val[0] / 333333.0;



        if(alliance == AllianceColor.BLUE){
            if (leftColor < threshold) {
                // left zone has it (flipped bc upside down)
                location = PropLocation.RIGHT;
                telemetry.addData("leftColor", leftColor);
                telemetry.addData("centerColor", centerColor);
                telemetry.addData("zone", "RIGHT");
                telemetry.update();
                Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255));
            } else if (centerColor < threshold) {
                // center zone has it
                location = PropLocation.CENTER;
                telemetry.addData("leftColor", leftColor);
                telemetry.addData("centerColor", centerColor);
                telemetry.addData("zone", "CENTER");
                telemetry.update();
                Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255));
            } else {
                // right zone has it (flipped bc upside down)
                location = PropLocation.LEFT;
                telemetry.addData("leftColor", leftColor);
                telemetry.addData("centerColor", centerColor);
                telemetry.addData("zone", "LEFT");
                telemetry.update();
                Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255));
            }
        }else{
            if (leftColor < threshold) {
                // left zone has it
                location = PropLocation.RIGHT;
                telemetry.addData("leftColor", leftColor);
                telemetry.addData("centerColor", centerColor);
                telemetry.addData("zone", "RIGHT");
                telemetry.update();
                Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255));
            } else if (centerColor < threshold) {
                // center zone has it
                location = PropLocation.CENTER;
                telemetry.addData("leftColor", leftColor);
                telemetry.addData("centerColor", centerColor);
                telemetry.addData("zone", "CENTER");
                telemetry.update();
                Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255));
            } else {
                // right zone has it
                location = PropLocation.LEFT;
                telemetry.addData("leftColor", leftColor);
                telemetry.addData("centerColor", centerColor);
                telemetry.addData("zone", "LEFT");
                telemetry.update();
                Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255));
            }
        }

        //Imgproc.rectangle(finalMat, leftZoneArea, new Scalar(255, 255, 255));
        Imgproc.rectangle(finalMat, centerZoneArea, new Scalar(255, 255, 255));
        Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255));
        Imgproc.rectangle(frame, centerZoneArea, new Scalar(255, 255, 255));


        Bitmap b = Bitmap.createBitmap(finalMat.width(), finalMat.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(finalMat, b);
        lastFrame.set(b);

        leftZone.release();
        centerZone.release();

        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public PropLocation getLocation() {
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
