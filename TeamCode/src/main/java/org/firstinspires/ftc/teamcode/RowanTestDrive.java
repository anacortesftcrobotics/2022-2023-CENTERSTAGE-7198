package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class RowanTestDrive extends OpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor viperSlide;
    DcMotor hookArm;
    Servo hookElbow;
    BNO055IMU imu;
    DigitalChannel bucketStop;
    //ColorRangeSensor floorTape;
    boolean elbow;
    boolean oldCircle;
    boolean oldRTrigger;
    boolean viperA;

    @Override
    public void init() {
        //floorTape = hardwareMap.get(ColorRangeSensor.class, "colorSensor");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        viperSlide = hardwareMap.get(DcMotor.class, "viperSlide");
        hookArm = hardwareMap.get(DcMotor.class, "hookArm");
        hookElbow = hardwareMap.get(Servo.class, "hookElbow");
        bucketStop = hardwareMap.get(DigitalChannel.class, "bucketStop");
        bucketStop.setMode(DigitalChannel.Mode.INPUT);



        //reverses direction of wheels (left)
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        hookArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hookArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        mecanumX();
        armControl();
        viper();
        telemetry.addData("PPR", hookArm.getCurrentPosition());
        //telemetry.addData("Red", floorTape.red());
        //telemetry.addData("Blue", floorTape.blue());
        //telemetry.addData("Green", floorTape.green());
        telemetry.addData("bucketStop", bucketStop.getState());
        telemetry.update();

    }

    private void viper() {
        //double taco = -gamepad2.left_stick_y;
        telemetry.addData("ViperP", viperSlide.getCurrentPosition());
        boolean bucketLimit = !bucketStop.getState();
        boolean viperToggle = gamepad2.left_bumper;
        if (bucketLimit) {
            viperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (viperToggle && !oldRTrigger) {
            viperA = !viperA;
            if (viperA) {
                viperSlide.setTargetPosition(-1800);
                viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                viperSlide.setPower(-0.7);
            } else {
                viperSlide.setTargetPosition(0);
                viperSlide.setPower(0.7);
            }
            //viperSlide.setPower(taco);

        }



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

        //does math for mechanim chassis
        frontLeft.setPower((forwards + sideways + rotate) / denominator);
        frontRight.setPower((forwards - sideways - rotate) / denominator);
        backLeft.setPower((forwards - sideways + rotate) / denominator);
        backRight.setPower((forwards + sideways - rotate) / denominator);

        telemetry.addData("FL", frontLeft.getPower());
    }

    private void armControl() {
        double position = hookArm.getCurrentPosition();
        double arm = -gamepad2.right_stick_y;
        /*boolean elbowToggle = gamepad2.dpad_up;
        if (elbowToggle && !oldCircle) {
            elbow = !elbow;
            if (elbow) {
                hookElbow.setPosition(0);
                hookArm.setTargetPosition(2650);
                hookArm.setPower(0.7);
            } else {
                hookArm.setTargetPosition(0);
                hookArm.setPower(-0.7);
            }
        }
        if (gamepad2.dpad_down) {
            hookElbow.setPosition(1);
        }
*/
        hookArm.setPower(arm);
        //oldCircle = elbowToggle;
    }
}

