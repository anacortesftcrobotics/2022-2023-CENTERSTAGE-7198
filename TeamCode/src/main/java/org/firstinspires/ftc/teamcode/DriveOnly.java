package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DriveOnly extends OpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor encoderLeft;
    DcMotor encoderRight;
    DcMotor encoderCenter;
    Odo1 odo;
    BNO055IMU imu;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        encoderLeft = frontLeft;
        encoderRight = backRight;
        encoderCenter = frontRight;
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        odo = new Odo1(11, 28.5, 4.9, 8192);

    }

    @Override
    public void loop() {
        mecanumX();
        Odometry();
        telemetry.update();
    }

    private void Odometry(){
        odo.setEncoderPos(encoderLeft.getCurrentPosition(),encoderRight.getCurrentPosition(),encoderCenter.getCurrentPosition());
        telemetry.addData("OdoX", odo.getX());
        telemetry.addData("OdoY", odo.getY());
        telemetry.addData("OdoR", odo.getHRad());
    }
    private void mecanumX() {
        double forwards = gamepad1.left_stick_y;
        double sideways = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(forwards) + Math.abs(sideways) + Math.abs(rotate), 1);

        telemetry.addData("Forward", forwards);

        //does math for mechanim chassis
        frontLeft.setPower((forwards + sideways + rotate) / denominator);
        frontRight.setPower((forwards + sideways - rotate) / denominator);
        backLeft.setPower((forwards - sideways + rotate) / denominator);
        backRight.setPower((forwards - sideways - rotate) / denominator);

        telemetry.addData("FL", frontLeft.getPower());
    }
}