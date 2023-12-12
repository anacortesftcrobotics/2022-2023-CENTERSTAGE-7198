package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

@Autonomous(name="AlfalfaAutoByTimeBackBlueBoardPlace", group="Robot")
public class AlfalfaAutoByTimeBackBlueBoardPlace extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor viperSlide;
    Servo bucketServo;
    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private ElapsedTime runtime = new ElapsedTime();
    boolean toggle;
    DigitalChannel bucketStop;
    boolean oldBucketLimit;
    boolean bucketToggle;



    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;
    double startTime = System.currentTimeMillis();
    public void resetTimer(){
        startTime = System.currentTimeMillis();
    }
    public double deltaTimeGet(){
        double deltaTime = System.currentTimeMillis() - startTime;
        //startTime = System.currentTimeMillis();
        return deltaTime;
    }
    private void viperPlace(int target)
    {
        /*boolean bucketLimit = !bucketStop.getState();
         boolean viperToggle = gamepad2.triangle;
         if (bucketLimit && !oldBucketLimit)
         {
         viperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         viperSlide.setTargetPosition(-5);
         viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         viperSlide.setPower(0.7);
         }

         resetTimer();
        while(opModeIsActive() && deltaTimeGet() <= 1750) {

        }
        toggle = true;
        viperSlide.setTargetPosition(-target);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(-0.7);

        resetTimer();
        while(opModeIsActive() && deltaTimeGet() <= 1750) {

        }
        bucketServo.setPosition(0.1);
        resetTimer();
        while(opModeIsActive() && deltaTimeGet() <= 3500) {

        }
        resetTimer();
        bucketServo.setPosition(0);
        viperSlide.setTargetPosition(0);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(0.7);
        while(opModeIsActive() && deltaTimeGet() <= 1750 ) {

        }*/
    }

    public void mecanumX(double forwards,double sideways, double rotate) {
        double denominator = Math.max(Math.abs(forwards) + Math.abs(sideways) + Math.abs(rotate), 1);

        telemetry.addData("Forward", forwards);

        //does math for mechanim chassis
        frontLeft.setPower((forwards + sideways + rotate) / denominator);
        frontRight.setPower((forwards - sideways - rotate) / denominator);
        backLeft.setPower((forwards - sideways + rotate) / denominator);
        backRight.setPower((forwards + sideways - rotate) / denominator);
    }

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        viperSlide = hardwareMap.get(DcMotor.class, "viperSlide");
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        bucketStop = hardwareMap.get(DigitalChannel.class, "bucketStop");
        initAprilTag();

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        bucketStop.setMode(DigitalChannel.Mode.INPUT);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        resetTimer();
        mecanumX(-0.45,0,0);
        while(opModeIsActive() && deltaTimeGet() <= 1000) {

        }
        resetTimer();
        mecanumX(0,0,0.45);
        while(opModeIsActive() && deltaTimeGet() <= 750) {

        }
        resetTimer();
        mecanumX(0,0,0);
        while(opModeIsActive() && deltaTimeGet() <= 1000) {

        }
        resetTimer();
        mecanumX(0.45,0,0);
        while(opModeIsActive() && deltaTimeGet() <= 1000) {

        }
        resetTimer();
        mecanumX(0,0,0);
        while(opModeIsActive() && deltaTimeGet() <= 1000) {

        }

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop
        resetTimer();
        telemetry.update();
        while(opModeIsActive() && deltaTimeGet() <= 10000) {

        }

        viperPlace(750);
        //previous: (0.75,0,0.25)
    }


    // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

    // Step 1:  Drive forward for 3 seconds


    // Step 2:  Spin right for 1.3 seconds
    /**leftDrive.setPower(TURN_SPEED);
     rightDrive.setPower(-TURN_SPEED);
     runtime.reset();
     while (opModeIsActive() && (runtime.seconds() < 1.3)) {
     telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
     telemetry.update();
     }

     // Step 3:  Drive Backward for 1 Second
     leftDrive.setPower(-FORWARD_SPEED);
     rightDrive.setPower(-FORWARD_SPEED);
     runtime.reset();
     while (opModeIsActive() && (runtime.seconds() < 1.0)) {
     telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
     telemetry.update();
     }*/

    // Step 4:  Stop
    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolut
        //
        //
        //
        // (new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();


        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()
}
