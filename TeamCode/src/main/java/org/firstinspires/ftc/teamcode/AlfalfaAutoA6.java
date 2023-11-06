package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AlfalfaAutoA4", group="robot")
public class AlfalfaAutoA6 extends LinearOpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    //DcMotorEx viperSlide;
    DcMotor viperSlide;
    DcMotor hookArm;
    //DcMotorEx intakeShoulder;
    DcMotor intakeShoulder;
    //DcMotorEx intakeElbow;
    DcMotor intakeElbow;
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
    boolean oldDpadRight;
    boolean oldCircle;
    boolean oldDpadUp;
    boolean oldYButton;
    boolean viperA;
    boolean fingerToggle;
    boolean oldXButton;
    boolean oldTrigger;
    private ElapsedTime runtime = new ElapsedTime();

    Odo1 odo;
    private void mecanumX(double forwards, double sideways, double rotate) {
        //double forwards = 0;
        //double sideways = 0.5;
        //double rotate = 0;
        double denominator = Math.max(Math.abs(forwards) + Math.abs(sideways) + Math.abs(rotate), 1);


        //does math for mechanim chassis
        frontLeft.setPower((forwards + sideways + rotate) / denominator);
        frontRight.setPower((forwards - sideways - rotate) / denominator);
        backLeft.setPower((forwards - sideways + rotate) / denominator);
        backRight.setPower((forwards + sideways - rotate) / denominator);
    }
    @Override
    public void runOpMode() {

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
        intakeElbow = hardwareMap.get(DcMotor.class, "grabWrist");
        //intakeShoulder = hardwareMap.get(DcMotorEx.class, "grabElbow");
        intakeShoulder = hardwareMap.get(DcMotor.class, "grabElbow");
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
        intakeShoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeElbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive() && (runtime.seconds() < 20)) {
                mecanumX(0,0,0);

        }
        while (opModeIsActive() && (runtime.seconds() > 20)) {
            mecanumX(0,0,0);
        }
    }
}