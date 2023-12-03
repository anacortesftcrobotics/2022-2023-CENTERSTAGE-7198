package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
public class VisionTest extends OpMode {

    // Field definitions
    VisionPortal visionPortal;
    Team7198PropProcessor visProcessor;

    // Initialization
    @Override
    public void init() {

        visProcessor = new Team7198PropProcessor(true, this);

        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), visProcessor);

        //visionPortal.resumeStreaming();
        visionPortal.resumeLiveView();

        //FtcDashboard.getInstance().startCameraStream(visProcessor, 0);
    }

    // Looping sequence of events
    @Override
    public void loop() {

    }

    public void stop()
    {
        visionPortal.close();
    }

    public void recieveVisionInfo(int x)
    {
        telemetry.addLine("" + x);
        telemetry.update();
    }
}
