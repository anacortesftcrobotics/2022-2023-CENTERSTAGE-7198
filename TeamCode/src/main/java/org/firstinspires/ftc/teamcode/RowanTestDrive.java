package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class RowanTestDrive extends OpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor hookArm;
    Servo hookElbow;
    BNO055IMU imu;
    ColorRangeSensor floorTape;
    boolean elbow;
    boolean oldCircle;
    @Override
    public void init() {
        floorTape = hardwareMap.get(ColorRangeSensor.class, "colorSensor");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        hookArm = hardwareMap.get(DcMotor.class, "hookArm");
        hookElbow = hardwareMap.get(Servo.class, "hookElbow");
        //reverses direction of wheels (left)
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        mecanumX();
        armControl();
        telemetry.addData("Red", floorTape.red());
        telemetry.addData("Blue", floorTape.blue());
        telemetry.addData("Green", floorTape.green());
        telemetry.update();
    }

    private void mecanumX() {
        double forwards = -gamepad1.left_stick_y;
        double sideways = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(forwards) + Math.abs(sideways) + Math.abs(rotate), 1);

        telemetry.addData("Forward", forwards);

        //does math for mechanim chassis
        frontLeft.setPower((forwards + sideways + rotate) / denominator);
        frontRight.setPower((forwards - sideways - rotate)  / denominator);
        backLeft.setPower((forwards - sideways + rotate)  / denominator);
        backRight.setPower((forwards + sideways - rotate)  / denominator);

        telemetry.addData("FL", frontLeft.getPower());
    }
    private void armControl(){
        double arm = -gamepad2.right_stick_y;
        boolean elbowToggle = gamepad2.circle;
        if (elbowToggle && !oldCircle){
            elbow = !elbow;
        }
        if (elbow){
            hookElbow.setPosition(0);
        }
        else {
            hookElbow.setPosition(1);
        }
        hookArm.setPower(arm);
        oldCircle = elbowToggle;
    }

