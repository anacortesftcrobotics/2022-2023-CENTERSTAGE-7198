package org.firstinspires.ftc.teamcode;

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

        visProcessor = new Team7198PropProcessor();

        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), visProcessor);

        //visionPortal.resumeStreaming();
        visionPortal.resumeLiveView();
    }

    // Looping sequence of events
    @Override
    public void loop() {

    }

    public void stop()
    {
        visionPortal.close();
    }
}
