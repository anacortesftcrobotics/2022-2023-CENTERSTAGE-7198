package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class RowanTestDrive extends OpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor hookArm;
    BNO055IMU imu;
    ColorRangeSensor floorTape;
    @Override
    public void init() {
        floorTape = hardwareMap.get(ColorRangeSensor.class, "colorSensor");
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");
        hookArm = hardwareMap.get(DcMotor.class, "hookArm");
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

        hookArm.setPower(arm);
    }
}
