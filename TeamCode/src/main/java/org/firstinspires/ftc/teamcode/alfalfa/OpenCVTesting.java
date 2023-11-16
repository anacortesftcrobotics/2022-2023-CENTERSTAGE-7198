package org.firstinspires.ftc.teamcode.alfalfa;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
@Autonomous(name="AlfalfaAutoA4", group="robot")
public class OpenCVTesting extends LinearOpMode {
    private VisionPortal visionPortal;
    VisionProcessor AlfalfaPropProcessor;
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        while (opModeIsActive() && (getRuntime() < 1)) {
            telemetry.update();
            VisionPortal.Builder builder = new VisionPortal.Builder();
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
            builder.addProcessor(AlfalfaPropProcessor);
            visionPortal = builder.build();
            visionPortal.resumeStreaming();
            sleep(50);

        }
    }
}
