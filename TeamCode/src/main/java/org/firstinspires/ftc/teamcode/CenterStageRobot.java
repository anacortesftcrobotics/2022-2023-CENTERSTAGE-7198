package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class CenterStageRobot {

    DcMotor frontLeft, frontRight, backLeft, backRight;
    DcMotor viperSlide;
    DcMotor hookArm;
    Servo intakeElbow;
    DcMotor encoderLeft, encoderRight, encoderCenter;
    Servo hookElbow;
    Servo fingerLf;
    Servo fingerRf;
    Servo shoulderServo;
    Servo bucketServo;

    public CenterStageRobot(CenterStageAutoBackstage.ALLIANCE thisAlliance, HardwareMap hardwareMap)
    {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        viperSlide = hardwareMap.get(DcMotor.class, "viperSlide");
        hookArm = hardwareMap.get(DcMotor.class, "hookArm");
        hookElbow = hardwareMap.get(Servo.class, "hookElbow");
        fingerLf = hardwareMap.get(Servo.class, "fingerLeft");
        fingerRf = hardwareMap.get(Servo.class, "fingerRight");
        intakeElbow = hardwareMap.get(Servo.class, "intakeWrist");
        shoulderServo = hardwareMap.get(Servo.class, "shoulder");
        bucketServo = hardwareMap.get(Servo.class, "bucketServo");

        // odometry wheels
        encoderLeft = backRight;
        encoderCenter = backLeft;
        encoderRight = frontLeft;

        //reverses direction of wheels (left)
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        hookArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hookArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // used in Auto to intake the pixels
    public void grabInitialPixels()
    {
        // FIXME: generalize to a teleop automation
        // FIXME: replace with autograb that uses sensors
        fingerRf.setPosition(1);
        fingerRf.setPosition(0);
        roboNap(500);
        fingerLf.setPosition(0);
        fingerLf.setPosition(1);
        roboNap(1500);

        setIntakeToCameraViewing();
    }

    public void placePurplePixel()
    {
        // down
        intakeElbow.setPosition(.6);
        shoulderServo.setPosition(.1);
        roboNap(500);

        // release
        fingerLf.setPosition(0);
        roboNap(1500);

        // up
        shoulderServo.setPosition(0.4);
        roboNap(500);
        hookElbow.setPosition(0);
        intakeElbow.setPosition(0.13);
    }

    public void halt()
    {
        mecanumX(0, 0, 0);
        roboNap(50);
    }

    public void setIntakeToCameraViewing()
    {
//        shoulderServo.setPosition(0.3);
        shoulderServo.setPosition(0.42);
        roboNap(100);
        intakeElbow.setPosition(0.13); // bump up a little higher to help the camera see
//        intakeElbow.setPosition(0.43);
        roboNap(400);
    }
    public void setIntakeToAutoScore()
    {
        shoulderServo.setPosition(0.42);
        roboNap(100);
        intakeElbow.setPosition(0.43);
        roboNap(400);
    }

    public void setIntakeToBatteringRam()
    {
        // use the intake elbow as a battering ram to move the prop out of the way
        intakeElbow.setPosition(.83);
        shoulderServo.setPosition(.3);
        roboNap(100);
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);
    }
    public void mecanumX(double forwards,double sideways, double rotate) {
        double denominator = Math.max(Math.abs(forwards) + Math.abs(sideways) + Math.abs(rotate), 1);

        double adjustedSidePowerFront = (forwards + sideways + rotate) / denominator;
        double adjustedSidePowerBack = (forwards - sideways + rotate) / denominator;

        // TODO: fix or remove this once the robot's weight is more balanced
        if (forwards != 0) {
            adjustedSidePowerFront += .1;
            adjustedSidePowerBack += .1;
        }

        //does math for mecanum chassis
        frontLeft.setPower(adjustedSidePowerFront);
        backLeft.setPower(adjustedSidePowerBack);
        frontRight.setPower((forwards - sideways - rotate) / denominator);
        backRight.setPower((forwards + sideways - rotate) / denominator);
    }

    public void drive(double forwards, double sideways, double rotate, int sleepMillis)
    {
        mecanumX(forwards, sideways, rotate);
        if (sleepMillis > 0) {
            roboNap(sleepMillis);
        }
    }
    public void roboNap(int millis)
    {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public void closeRightFinger()
    {
        fingerRf.setPosition(1);
        roboNap(1000);
    }

}
