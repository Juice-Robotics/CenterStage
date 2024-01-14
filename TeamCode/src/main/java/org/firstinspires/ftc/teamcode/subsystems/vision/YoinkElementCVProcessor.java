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


    public static double redThreshold = 150;
    public static double blueThreshold = 70;
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
        Rect leftZoneArea;
        Rect centerZoneArea;

        if (alliance == AllianceColor.RED) {
            leftZoneArea = new Rect(redLeftX, redLeftY, width, height);
            centerZoneArea = new Rect(redCenterX, redCenterY, width, height);
//            leftZoneArea = new Rect(redLeftX, redLeftY, leftWidth, leftHeight);
//            centerZoneArea = new Rect(redCenterX, redCenterY, centerWidth, centerHeight);
        } else {
            leftZoneArea = new Rect(blueLeftX, blueLeftY, width, height);
            centerZoneArea = new Rect(blueCenterX, blueCenterY, width, height);
        }

        Mat leftZone = frame.submat(leftZoneArea);
        Mat centerZone = frame.submat(centerZoneArea);


//        if (DEBUG) {
//            Imgproc.blur(frame, frame, new Size(5, 5));
//            Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255), 2);
//            Imgproc.rectangle(frame, centerZoneArea, new Scalar(255, 255, 255), 2);
//        }

        Imgproc.blur(leftZone, leftZone, new Size(5, 5));
        Imgproc.blur(centerZone, centerZone, new Size(5, 5));

        left = Core.mean(leftZone);
        center = Core.mean(centerZone);

        if (telemetry != null) {
            telemetry.addData("leftColor", left.toString());
            telemetry.addData("centerColor", center.toString());
            telemetry.addData("analysis", location.toString());
            telemetry.update();
        }

        double threshold = alliance == AllianceColor.RED ? redThreshold : blueThreshold;
        int idx = alliance == AllianceColor.RED ? 0 : 2;

        leftColor = left.val[idx];
        centerColor = center.val[idx];

        if (leftColor > threshold && (left.val[0] + left.val[1] + left.val[2] - left.val[idx] < left.val[idx])) {
            // left zone has it
            location = PropLocation.LEFT;
            Imgproc.rectangle(frame, leftZoneArea, new Scalar(255, 255, 255), 10);
        } else if (centerColor > threshold && (center.val[0] + center.val[1] + center.val[2] - center.val[idx] < center.val[idx])) {
            // center zone has it
            location = PropLocation.CENTER;
            Imgproc.rectangle(frame, centerZoneArea, new Scalar(255, 255, 255), 10);
        } else {
            // right zone has it
            location = PropLocation.RIGHT;
        }

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
