package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.*;

@TeleOp
public class RowanTestDrive extends OpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    //DcMotorEx viperSlide;
    DcMotor viperSlide;
    DcMotor hookArm;
    //DcMotorEx intakeShoulder;
    DcMotorEx intakeShoulder;
    //DcMotorEx intakeElbow;
    Servo intakeElbow;
    DcMotor encoderLeft;
    DcMotor encoderRight;
    DcMotor encoderCenter;

    Servo hookElbow;
    Servo fingerRight;
    Servo fingerLeft;
    Servo bucketServo;
    BNO055IMU imu;
    DigitalChannel bucketStop;
    boolean oldBucketLimit;

    //ColorRangeSensor floorTape;
    boolean elbow;
    boolean bucket;
    boolean oldSquare;
    boolean fingers;
    boolean intake;
    boolean oldTriangle;
    boolean oldCircle;
    boolean oldDpadUp;
    boolean oldYButton;
    boolean viperA;
    boolean fingerToggle;
    boolean oldXButton;
    boolean oldTrigger;
    Odo1 odo;

    @Override
    public void init() {
        //floorTape = hardwareMap.get(ColorRangeSensor.class, "colorSensor");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        //viperSlide = hardwareMap.get(DcMotorEx.class, "viperSlide");
        viperSlide = hardwareMap.get(DcMotor.class, "viperSlide");
        //hookArm = hardwareMap.get(DcMotorEx.class, "hookArm");
        hookArm = hardwareMap.get(DcMotor.class, "hookArm");
        hookElbow = hardwareMap.get(Servo.class, "hookElbow");
        bucketStop = hardwareMap.get(DigitalChannel.class, "bucketStop");
        fingerRight = hardwareMap.get(Servo.class, "fingerRight");
        fingerLeft = hardwareMap.get(Servo.class, "fingerLeft");
        // intakeElbow = hardwareMap.get(DcMotorEx.class, "grabWrist");
        intakeElbow = hardwareMap.get(Servo.class, "intakeWrist");
        //intakeShoulder = hardwareMap.get(DcMotorEx.class, "grabElbow");
        intakeShoulder = hardwareMap.get(DcMotorEx.class, "grabElbow");
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");
        bucketStop.setMode(DigitalChannel.Mode.INPUT);

        encoderLeft = backRight;
        encoderCenter = backLeft;
        encoderRight = frontLeft;

        encoderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderCenter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odo = new Odo1(11, 28.5, 4.7, 8192);
        hookElbow.setPosition(0.5);



        //reverses direction of wheels (left)
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        hookArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hookArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeShoulder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //viperSlide.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(0,0,0,0));
        intakeShoulder.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(0,0,0,0));

    }

    @Override
    public void loop() {
        mecanumX();
        armControl();
        viper();
        intakeControl();
        fingerControl();
        Odometry();
        openBucket();
        telemetry.addData("PPR", hookArm.getCurrentPosition());
        telemetry.addData("bucketStop", bucketStop.getState());
        telemetry.update();

    }
    private void Odometry(){
        odo.setEncoderPos(-encoderLeft.getCurrentPosition(),-encoderRight.getCurrentPosition(),-encoderCenter.getCurrentPosition());
        telemetry.addData("EncoderR", -encoderRight.getCurrentPosition());
        telemetry.addData("EncoderL", -encoderLeft.getCurrentPosition());
        telemetry.addData("EncoderC", -encoderCenter.getCurrentPosition());
        telemetry.addData("OdoX", odo.getX());
        telemetry.addData("OdoY", odo.getY());
        telemetry.addData("OdoR", odo.getHRad());
    }
    private void viper() {
        /** viper input: triangle**/
        //double taco = -gamepad2.left_stick_y;
        telemetry.addData("ViperP", viperSlide.getCurrentPosition());
        boolean bucketLimit = !bucketStop.getState();
        boolean viperToggle = gamepad2.triangle;
        if (bucketLimit && !oldBucketLimit) {
            viperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            viperSlide.setTargetPosition(-200);
            viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            viperSlide.setPower(0.7);
        }
        if (viperToggle && !oldTriangle) {
            viperA = !viperA;
            if (viperA) {
                viperSlide.setTargetPosition(-1500);
                viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viperSlide.setPower(-0.7);
            } else {
                viperSlide.setTargetPosition(0);
                viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viperSlide.setPower(0.7);
            }
            //viperSlide.setPower(taco);

        }


        oldTriangle = viperToggle;
        oldBucketLimit = bucketLimit;
    }


    /*private void initAprilTag() {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                .build();

       // VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(aprilTag);
        visionPortal = builder.build();

    }*/

    private void mecanumX() {
        double forwards = -gamepad1.left_stick_y;
        double sideways = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(forwards) + Math.abs(sideways) + Math.abs(rotate), 1);

        telemetry.addData("Forward", forwards);
        telemetry.addData("shoulder",intakeShoulder.getCurrentPosition());

        //does math for mechanim chassis
        frontLeft.setPower((forwards + sideways + rotate) / denominator);
        frontRight.setPower((forwards - sideways - rotate) / denominator);
        backLeft.setPower((forwards - sideways + rotate) / denominator);
        backRight.setPower((forwards + sideways - rotate) / denominator);
    }

    private void armControl() {
        /** armControl input: dpad up, right stick Y **/
        double position = hookArm.getCurrentPosition();
        double arm = -gamepad2.right_stick_y;
        boolean elbowToggle = gamepad2.dpad_up;
        if (elbowToggle && !oldDpadUp) {
            elbow = !elbow;

            //if (elbow) {

            //}
        //if (gamepad1.dpad_right)
                //hookElbow.setPosition(1);
                //hookArm.setTargetPosition(200);
                //hookArm.setPower(0.7);
            } //else {
                //hookArm.setTargetPosition(0);
                //hookArm.setPower(-1);
            //}
        //}
        ///if (gamepad2.) {
    //    }
        hookElbow.setPosition(elbow ? 0.5d : 0.0d);

        hookArm.setPower(arm);
        //oldTriangle = elbowToggle;
    }
    private void intakeControl() {
        /** intakeControl input: circle **/
        boolean intakeToggle = gamepad2.circle;
        if (intakeToggle && !oldCircle) {
            intake = !intake;
            if (intake) {
                intakeElbow.setPosition(0.75);
                intakeShoulder.setTargetPosition(-1300);
                intakeShoulder.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                //intakeShoulder.setTargetPosition(-1000);
                intakeShoulder.setPower(-0.4);
            }
            else {
                intakeShoulder.setPower(0.4);
                intakeShoulder.setTargetPosition(0);
                intakeElbow.setPosition(0);
            }
        }
        oldCircle = intakeToggle;
    }
    private void fingerControl() {
        /** fingerControl input: X **/
        boolean fingerToggle = gamepad2.x;
        if (fingerToggle && !oldXButton) {
            fingers = !fingers;
            if (fingers) {
                fingerRight.setPosition(0);
                fingerLeft.setPosition(1);
            } else {
                fingerRight.setPosition(1);
                fingerLeft.setPosition(0);
            }
        }
        oldXButton = fingerToggle;
    }
    private void openBucket() {
        /** openBucket input: square **/
        boolean bucketToggle = gamepad2.square;
        if (bucketToggle && !oldSquare) {
            bucket = !bucket;
            if (bucket) {
                bucketServo.setPosition(0.1);
            }
            else {
                bucketServo.setPosition(0);
            }
        }
        oldSquare = bucketToggle;
    }
}

