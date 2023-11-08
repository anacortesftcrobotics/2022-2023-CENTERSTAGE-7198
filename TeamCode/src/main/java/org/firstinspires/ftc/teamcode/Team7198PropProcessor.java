package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
import org.opencv.objdetect.*;
import org.opencv.features2d.*;
import org.opencv.imgproc.*;

import java.util.ArrayList;
import java.util.List;

public class Team7198PropProcessor implements VisionProcessor
{
    private CameraCalibration cCal;
    private Rect[] boundRect;
    private Paint rectPaint;

    @Override
    public void init(int width, int height, CameraCalibration calibration)
    {
        cCal = calibration;

        rectPaint = new Paint();
        rectPaint.setAntiAlias(true);
        rectPaint.setColor(Color.rgb(12, 145, 201));
        rectPaint.setStyle(Paint.Style.FILL);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        Mat gray = new Mat(frame.rows(),frame.cols(),frame.type());
        Mat edges = new Mat(frame.rows(),frame.cols(),frame.type());
        Mat cannyOutput = new Mat(frame.rows(),frame.cols(),frame.type(), new Scalar(0));
        Imgproc.cvtColor(frame, gray, Imgproc.COLOR_BGR2GRAY);

        Imgproc.blur(gray,edges,new Size(3,3));
        Imgproc.Canny(edges,edges,100,300);
        frame.copyTo(cannyOutput,edges);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(cannyOutput, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        boundRect = new Rect[contours.size()];
        Point[] centers = new Point[contours.size()];
        float[][] radius = new float[contours.size()][1];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
            centers[i] = new Point();
            Imgproc.minEnclosingCircle(contoursPoly[i], centers[i], radius[i]);
        }

        //Mat drawing = Mat.zeros(cannyOutput.size(), CvType.CV_8UC3);

        return null;
    }


    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {


        canvas.drawRect(
                (float)boundRect[0].tl().x * scaleBmpPxToCanvasPx,
                (float)boundRect[0].tl().y * scaleBmpPxToCanvasPx,
                (float)boundRect[0].br().x * scaleBmpPxToCanvasPx,
                (float)boundRect[0].br().y * scaleBmpPxToCanvasPx,
                rectPaint);
    }
}
