package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.logancode.*;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.*;
import org.opencv.objdetect.*;
import org.opencv.features2d.*;
import org.opencv.imgproc.*;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class Team7198PropProcessor implements VisionProcessor, CameraStreamSource
{
    private CameraCalibration cCal;
    private Rect[] boundRect;
    private Paint rectPaint;

    public int data;
    private boolean isRed;
    private Object origin;

    private final AtomicReference<Bitmap> lastFrame =
            new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    public Team7198PropProcessor(boolean isRed, Object origin)
    {
        this.isRed = isRed;
        this.origin = origin;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration)
    {
        cCal = calibration;

        rectPaint = new Paint();
        rectPaint.setAntiAlias(true);
        rectPaint.setColor(Color.rgb(12, 255, 12));
        rectPaint.setStyle(Paint.Style.STROKE);
        rectPaint.setStrokeWidth(5);

        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        Mat ourFrame = new Mat(frame.rows(),frame.cols(), frame.type());
        Mat mask = new Mat(frame.rows(),frame.cols(), CvType.CV_8U);
        Mat maskedFrame = new Mat(frame.rows(),frame.cols(),frame.type());
        Mat bgrMask = new Mat(frame.rows(),frame.cols(),frame.type());
        Imgproc.cvtColor(frame, ourFrame, Imgproc.COLOR_RGB2HSV);

        if(isRed)
        {
            Scalar lower_red = new Scalar(0, 150, 100);
            Scalar upper_red = new Scalar(20, 255, 255);

            Core.inRange(ourFrame, lower_red, upper_red, mask);
        }
        else {
            Scalar lower_blue = new Scalar(106, 150, 100);
            Scalar upper_blue = new Scalar(146, 255, 255);

            Core.inRange(ourFrame, lower_blue, upper_blue, mask);
        }

        Imgproc.cvtColor(mask, bgrMask, Imgproc.COLOR_GRAY2RGBA);

        Core.addWeighted(frame, 0.7, bgrMask, 0.3, 0, maskedFrame);

        //Imgproc.cvtColor(maskedFrame, maskedFrame, Imgproc.COLOR_BGR2RGB);

        Bitmap b = Bitmap.createBitmap(maskedFrame.width(), maskedFrame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(maskedFrame, b);
        lastFrame.set(b);

        //Imgproc.blur(mask,mask,new Size(3,3));
        //Imgproc.Canny(mask,edges,100,300);
        //frame.copyTo(cannyOutput,edges);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

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

        int largestSize = 800; //Minimum Threshold
        int index = -1;
        for(int i = 0; i < boundRect.length; i++)
        {
            int currentSize = boundRect[i].width * boundRect[i].height;
            if(currentSize > largestSize)
            {
                largestSize = currentSize;
                index = i;
            }
        }

        if(boundRect.length > 0 && index != -1)
        {
            int x = (boundRect[index].x + boundRect[index].width / 2) / (frame.width() / 2);
            x = Math.min(x, 1) + 1;
//            if(origin.getClass().equals(VisionTest.class))
//                ((VisionTest) origin).recieveVisionInfo(x);
//            if(origin.getClass().equals(PathFollower.class))
//                ((PathFollower) origin).recieveVisionInfo(x);
//            if(origin.getClass().equals(PathFollower.class))
//                ((Auto4F) origin).recieveVisionInfo(x);
//            if(origin.getClass().equals(PathFollower.class))
//                ((Auto2F) origin).recieveVisionInfo(x);
//            if(origin.getClass().equals(PathFollower.class))
//                ((Auto4A) origin).recieveVisionInfo(x);
//            if(origin.getClass().equals(PathFollower.class))
//                ((Auto2A) origin).recieveVisionInfo(x);

            data = x;
        }
        else
        {
//            if(origin.getClass().equals(VisionTest.class))
//                ((VisionTest) origin).recieveVisionInfo(0);
//            if(origin.getClass().equals(PathFollower.class))
//                ((PathFollower) origin).recieveVisionInfo(0);
//            if(origin.getClass().equals(PathFollower.class))
//                ((Auto4F) origin).recieveVisionInfo(0);
//            if(origin.getClass().equals(PathFollower.class))
//                ((Auto2F) origin).recieveVisionInfo(0);
//            if(origin.getClass().equals(PathFollower.class))
//                ((Auto4A) origin).recieveVisionInfo(0);
//            if(origin.getClass().equals(PathFollower.class))
//                ((Auto2A) origin).recieveVisionInfo(0);
            data = 0;
        }

        return null;
    }


    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        if(boundRect.length == 0)
            return;

        int largestSize = 0;
        int index = 0;
        for(int i = 0; i < boundRect.length; i++)
        {
            int currentSize = boundRect[i].width * boundRect[i].height;
            if(currentSize > largestSize)
            {
                largestSize = currentSize;
                index = i;
            }
        }

//        for(int i = 0; i < boundRect.length && i < 50; i++)
//        {
//            canvas.drawRect(
//                    (float) boundRect[i].tl().x * scaleBmpPxToCanvasPx,
//                    (float) boundRect[i].tl().y * scaleBmpPxToCanvasPx,
//                    (float) boundRect[i].br().x * scaleBmpPxToCanvasPx,
//                    (float) boundRect[i].br().y * scaleBmpPxToCanvasPx,
//                    rectPaint);
//        }

        canvas.drawRect(
            (float) boundRect[index].tl().x * scaleBmpPxToCanvasPx,
            (float) boundRect[index].tl().y * scaleBmpPxToCanvasPx,
            (float) boundRect[index].br().x * scaleBmpPxToCanvasPx,
            (float) boundRect[index].br().y * scaleBmpPxToCanvasPx,
            rectPaint);
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }
}
