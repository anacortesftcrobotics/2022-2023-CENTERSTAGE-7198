package org.firstinspires.ftc.teamcode.alfalfa;

import android.graphics.Canvas;
import android.graphics.Paint;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class AlfalfaPropProcessor implements VisionProcessor {

    private static Paint rectPaint;
    private final List<MatOfPoint> contours1 = new ArrayList<>();
    Mat scrHSV;
    private static final int MAX_THRESHOLD = 255;
    private Random rng = new Random(12345);
    private Rect[] boundRect;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        /**setting up paint for bounding**/
        rectPaint = new Paint();
        rectPaint.setARGB(1,0,255,0);
        rectPaint.setStrokeWidth(2);
        rectPaint.setStyle(Paint.Style.STROKE);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        /**search for certain color, changing to binary**/
        Mat inRangeDst = new Mat(frame.rows(), frame.cols(), CvType.CV_8U);
        Imgproc.cvtColor(frame, scrHSV, Imgproc.COLOR_RGB2HSV);

        Scalar inRangeLower = new Scalar(0,0,0);
        Scalar inRangeUpper = new Scalar(0,0,0);

        Core.inRange(scrHSV,inRangeLower,inRangeUpper,inRangeDst);

        /**finding contours**/
        Mat hierarchy = new Mat();
        Imgproc.findContours(inRangeDst, contours1, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours1.size()];
        Rect[] boundRect = new Rect[contours1.size()];;
        for (int i = 0; i < contours1.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours1.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect = new Rect[]{Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()))};

            return null;
        }
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Mat drawing = Mat.zeros(scrHSV.size(), CvType.CV_8U);
        for (int i = 0; i < contours1.size(); i++) {
            Scalar color = new Scalar(rng.nextInt(256), rng.nextInt(256), rng.nextInt(256));
            Imgproc.rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2);
        }

        canvas.drawRect (boundRect,rectPaint);

    }
}
