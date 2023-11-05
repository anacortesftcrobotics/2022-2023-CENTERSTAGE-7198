package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.objdetect.*;
import org.opencv.features2d.*;
import org.opencv.imgproc.*;

public class Team7198PropProcessor implements VisionProcessor
{
    private CameraCalibration cCal;

    @Override
    public void init(int width, int height, CameraCalibration calibration)
    {
        cCal = calibration;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        Mat gray = new Mat(frame.rows(),frame.cols(),frame.type());
        Mat edges = new Mat(frame.rows(),frame.cols(),frame.type());
        Mat dst = new Mat(frame.rows(),frame.cols(),frame.type(), new Scalar(0));
        Imgproc.cvtColor(frame, gray, Imgproc.COLOR_BGR2GRAY);

        Imgproc.blur(gray,edges,new Size(3,3));
        Imgproc.Canny(edges,edges,100,300);
        frame.copyTo(dst,edges);


        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
