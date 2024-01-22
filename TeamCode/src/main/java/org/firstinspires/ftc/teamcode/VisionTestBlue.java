package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
@Disabled
public class VisionTestBlue extends OpMode {

    // Field definitions
    double visionData;
    VisionPortal visionPortal;
    Team7198PropProcessor visProcessor;

    // Initialization
    @Override
    public void init() {

        visProcessor = new Team7198PropProcessor(false, this);

        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), visProcessor);

        //visionPortal.resumeStreaming();
        visionPortal.resumeLiveView();

        //FtcDashboard.getInstance().startCameraStream(visProcessor, 0);
    }

    // Looping sequence of events
    @Override
    public void loop() {
        visionData = visProcessor.data;
        telemetry.addLine("" + visionData);
        telemetry.update();
    }

    public void stop()
    {
        visionPortal.stopLiveView();
        //visionPortal.stopStreaming();
        visionPortal.close();
    }
}
