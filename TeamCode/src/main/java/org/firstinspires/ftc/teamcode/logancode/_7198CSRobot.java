package org.firstinspires.ftc.teamcode.logancode;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ThreadPool;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.kaicode.PIDFArmController;

import java.util.concurrent.ExecutorService;

public class _7198CSRobot {
    DigitalChannel bucketStop;
    Servo fingerRight, fingerLeft, wristServo, droneServo;
    DigitalChannel pixelSlideSwitch;
    DcMotor frontLeft, frontRight, backLeft, backRight, pixelBase, pixelSlide;
    DcMotor encoderLeft, encoderRight, encoderCenter; // odometry wheels
    PIDFArmController pidfArmController;
    double pixelArmToRadiansConstant = -1 / 50.9 * 4.5 * (Math.PI/180);

    public static double p = -1.3;
    public static double kg = -1;
    public static double i;
    public static double d;
    public static double kv = -1.5;
    public static double ka;
    private ExecutorService threadExecuter;
    private Runnable backgroundThread;
    private boolean viperThreadLock = false;
    public _7198CSRobot(HardwareMap hardwareMap)
    {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        pixelBase = hardwareMap.get(DcMotor.class, "pixelBase");
        pixelSlide = hardwareMap.get(DcMotor.class, "pixelSlide");

        fingerRight = hardwareMap.get(Servo.class, "servoA");
        fingerLeft = hardwareMap.get(Servo.class, "servoB");

        wristServo = hardwareMap.get(Servo.class, "wristServo");
        droneServo = hardwareMap.get(Servo.class, "droneServo");

        pixelBase = hardwareMap.get(DcMotor.class, "pixelBase");
        pixelBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pixelBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pixelSlide = hardwareMap.get(DcMotorEx.class, "pixelSlide");
        pixelSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pixelSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pixelSlideSwitch = hardwareMap.get(DigitalChannel.class, "PSSwitch");

        // odometry wheels
        encoderLeft = backRight;
        encoderCenter = backLeft;
        encoderRight = frontLeft;

        //reverses direction of wheels (left)
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        pidfArmController = new PIDFArmController(p,i,d,kv,ka,kg,0);
        //kg:-1
        //kv:-1.5
        //p:-1.2
        pidfArmController.launch(-15 *Math.PI/180,System.currentTimeMillis());

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

    //FIXME: Bro, this needs to happen every update! do looping control flow 💀
    public void SetArmAngle(Telemetry telemetry, double targetPosition) {
        targetPosition = targetPosition*Math.PI/180;
        telemetry.addData("target: ", targetPosition / Math.PI * 180);
        telemetry.addData("error: ", targetPosition -  (pixelBase.getCurrentPosition() * pixelArmToRadiansConstant));
        //50.9 to 1 ; 105*Math.PI/180
        double correction = pidfArmController.updateArm(targetPosition, (pixelBase.getCurrentPosition() * pixelArmToRadiansConstant), 0,System.currentTimeMillis());
        telemetry.addData("correction: ", correction);

        pixelBase.setPower(correction);
    }

    public void LimpArm()
    {
        pixelBase.setPower(0);
    }

    // used in Auto to intake the pixels
    public void grabInitialPixels()
    {
        // TODO: generalize to a teleop automation
        // TODO: replace with autograb that uses sensors

        //open fingers
        //fingerRight.setPosition(0.6);
        //fingerLeft.setPosition(0.54);

        //close fingers
        fingerRight.setPosition(1);
        fingerLeft.setPosition(0);
        roboNap(1500);

        //setIntakeToCameraViewing();
    }

    //returns true when completed
    public boolean placePurplePixel()
    {
        //this function will loop doe to external factors. this means we update the arms position and check if it is at the right angle
        //once it is at the right angle we can place
        //only then can we move on to the next code segments
        return false;

        // down
        //intakeElbow.setPosition(.6);
        //shoulderServo.setPosition(.1);
        //roboNap(500);

        // release
        //fingerLf.setPosition(0);
        //roboNap(1500);

        // up
        //shoulderServo.setPosition(0.4);
        //roboNap(500);
        //hookElbow.setPosition(0);
        //intakeElbow.setPosition(0.13);
    }
    public void halt()
    {
        mecanumX(0, 0, 0);
        roboNap(50);
    }
    @Deprecated
    public void setIntakeToCameraViewing()
    {
//        shoulderServo.setPosition(0.5);
//        roboNap(100);
//        intakeElbow.setPosition(0.13); // bump up a little higher to help the camera see
//        roboNap(400);
    }
    public void setIntakeToAutoScore()
    {
//        shoulderServo.setPosition(0.42);
//        roboNap(100);
//        intakeElbow.setPosition(0.43);
//        roboNap(400);
    }
    public void closeBothFingers()
    {
        fingerRight.setPosition(1);
        fingerLeft.setPosition(0);
    }
    public void setIntakeToBatteringRam()
    {
//        intakeElbow.setPosition(1);
//        shoulderServo.setPosition(0.32);

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
//        if (! viperThreadLock) {
//            viperSlide.setPower(power);
//        }
    }
    public void depositPixelsInBucketDragOff() // this is the code that actually loads the bucket
    {
//        roboNap(10);
//        shoulderServo.setPosition(0.86);
//        roboNap(800);
//        intakeElbow.setPosition(0);
//        roboNap(100);
//        closeBothFingers();
//        roboNap(100);

//        viperSlide.setPower(-.3);
//        roboNap(600); // into the bucket
//        viperSlide.setPower(0);
//        roboNap(100);

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
        //intakeElbow.setPosition(1);
        roboNap(200);
        //shoulderServo.setPosition(0.1);
        roboNap(300);
        fingerRight.setPosition(0);
        fingerLeft.setPosition(1);
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
//        double denominator = Math.max(Math.abs(forwards) + Math.abs(sideways) + Math.abs(rotate), 1);
//
//        double adjustedSidePowerFront = (forwards + sideways + rotate) / denominator;
//        double adjustedSidePowerBack = (forwards - sideways + rotate) / denominator;
//
//
//        // does math for mecanum chassis
//        frontLeft.setPower(adjustedSidePowerFront);
//        backLeft.setPower(adjustedSidePowerBack);
//        frontRight.setPower((forwards - sideways - rotate) / denominator);
//        backRight.setPower((forwards + sideways - rotate) / denominator);
    }

    // TODO: consolidate this with mecanumX
    // TODO: This is only used by the AprilTag drive-up code (came with the sample code).
    // TODO: We should only have one way to drive the robot.
    public void moveRobot(double x, double y, double rx) {
        double leftBackPower;
        double rightBackPower;
        double leftFrontPower;
        double rightFrontPower;

        //double x = gamepad1.left_stick_x;
        //double y = -gamepad1.left_stick_y;
        //double rx = gamepad1.right_stick_x;

        //updateSpeedCoefficient();

        x = LogsUtils.exponentialRemapAnalog(LogsUtils.deadZone(x,0.02),2);
        y = LogsUtils.exponentialRemapAnalog(LogsUtils.deadZone(y,0.02),2);
        rx = LogsUtils.exponentialRemapAnalog(LogsUtils.deadZone(rx,0.02),2);

        //telemetry.addData("Remapped Y: ", y);

        //rx += poleCenter();

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        leftBackPower   = (y - x + rx) / denominator;
        rightBackPower  = (y + x - rx) / denominator;
        leftFrontPower  = (y + x + rx) / denominator;
        rightFrontPower = (y - x - rx) / denominator;

        leftBackPower   = Math.cbrt(leftBackPower);
        rightBackPower  = Math.cbrt(rightBackPower);
        leftFrontPower  = Math.cbrt(leftFrontPower);
        rightFrontPower = Math.cbrt(rightFrontPower);

        leftBackPower   = (leftBackPower);
        rightBackPower  = (rightBackPower);
        leftFrontPower  = (leftFrontPower);
        rightFrontPower = (rightFrontPower);

        frontLeft.setPower(leftBackPower);
        backLeft.setPower(rightBackPower);
        frontRight.setPower(leftFrontPower);
        backRight.setPower(rightFrontPower);
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
        fingerLeft.setPosition(1);
        roboNap(1000);
    }

}

