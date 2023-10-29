package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;
//import org.opencv.

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


        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }
}
