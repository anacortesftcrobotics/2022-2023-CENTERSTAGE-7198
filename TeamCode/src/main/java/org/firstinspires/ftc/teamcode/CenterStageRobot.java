package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ThreadPool;

import java.util.concurrent.ExecutorService;

public class CenterStageRobot {
    DigitalChannel bucketStop;
    Servo bucketServo, shoulderServo, intakeElbow, hookElbow, fingerLf, fingerRf, launcherServo;
    DcMotor frontLeft, frontRight, backLeft, backRight, viperSlide, hookArm;
    DcMotor encoderLeft, encoderRight, encoderCenter; // odometry wheels
    private ExecutorService threadExecuter;
    private Runnable backgroundThread;
    private boolean viperThreadLock = false;
    public CenterStageRobot(HardwareMap hardwareMap)
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
        bucketStop = hardwareMap.get(DigitalChannel.class, "bucketStop");
        launcherServo = hardwareMap.get(Servo.class, "launcherServo");

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

        threadExecuter = ThreadPool.newSingleThreadExecutor("loadPixel");
        backgroundThread = () -> {
            try {
                viperThreadLock = true;
                this.depositPixelsInBucketDragOff();
                viperThreadLock = false;
            } catch (Throwable t) {
                // hm, should probably throw an error
            }
        };
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
    public void openBucket()
    {
        bucketServo.setPosition(0.6);
    }
    public void closeBucket()
    {
        bucketServo.setPosition(.8);
    }
    public void halt()
    {
        mecanumX(0, 0, 0);
        roboNap(50);
    }
    public void setIntakeToCameraViewing()
    {
        shoulderServo.setPosition(0.5);
        roboNap(100);
        intakeElbow.setPosition(0.13); // bump up a little higher to help the camera see
        roboNap(400);
    }
    public void setIntakeToAutoScore()
    {
        shoulderServo.setPosition(0.42);
        roboNap(100);
        intakeElbow.setPosition(0.43);
        roboNap(400);
    }
    public void closeBothFingers()
    {
        fingerRf.setPosition(1);
        fingerLf.setPosition(0);
    }
    public void setIntakeToBatteringRam()
    {
        intakeElbow.setPosition(1);
        shoulderServo.setPosition(0.32);
        roboNap(100);
    }

    // We transfer from intake to deposit in a background thread because it takes a while.
    // It needs to be accurate, but we should be able to move the drivetrain while its happening.
    public void loadPixelsInBucket()
    {
        threadExecuter.submit(backgroundThread);
    }

    public void safeViperSlide(double power)
    {
        if (! viperThreadLock) {
            viperSlide.setPower(power);
        }
    }
    public void depositPixelsInBucketDragOff() // this is the code that actually loads the bucket
    {
        roboNap(10);
        shoulderServo.setPosition(0.86);
        roboNap(800);
        intakeElbow.setPosition(0);
        roboNap(100);
        closeBothFingers();
        roboNap(100);

        viperSlide.setPower(-.3);
        roboNap(600); // into the bucket
        viperSlide.setPower(0);
        roboNap(100);

        setIntakeToBatteringRam();

        /*
        shoulderServo.setPosition(.6);
        intakeElbow.setPosition(.1);
        roboNap(500);
        viperSlide.setPower(.3);
*/
    }
    public void intakeDownGrabPixelsComeUp()
    {
       intakeElbow.setPosition(1);
        roboNap(200);
        shoulderServo.setPosition(0.1);
        roboNap(300);
        fingerRf.setPosition(0);
        fingerLf.setPosition(1);
        roboNap(700);
        setIntakeToBatteringRam();
    }

    // wrapper around mecanumX and roboNap, because commonly those are called together
    public void drive(double forwards, double sideways, double rotate, int sleepMillis)
    {
        mecanumX(forwards, sideways, rotate);
        if (sleepMillis > 0) {
            roboNap(sleepMillis);
        }
    }
    public void mecanumX(double forwards,double sideways, double rotate) {
        double denominator = Math.max(Math.abs(forwards) + Math.abs(sideways) + Math.abs(rotate), 1);

        double adjustedSidePowerFront = (forwards + sideways + rotate) / denominator;
        double adjustedSidePowerBack = (forwards - sideways + rotate) / denominator;

        // TODO: fix or remove this once the robot's weight is more balanced
        if (forwards > 0) {
            adjustedSidePowerFront += .1;
            adjustedSidePowerBack += .1;
        } else if (forwards < 0) {
            adjustedSidePowerFront -= .1;
            adjustedSidePowerBack -= .1;
        }

        // does math for mecanum chassis
        frontLeft.setPower(adjustedSidePowerFront);
        backLeft.setPower(adjustedSidePowerBack);
        frontRight.setPower((forwards - sideways - rotate) / denominator);
        backRight.setPower((forwards + sideways - rotate) / denominator);
    }

    // TODO: consolidate this with mecanumX
    // TODO: This is only used by the AprilTag drive-up code (came with the sample code).
    // TODO: We should only have one way to drive the robot.
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
