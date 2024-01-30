package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.kaicode.PIDFArmController;
import org.firstinspires.ftc.teamcode.logancode.LogsUtils;

@TeleOp
@Config
public class CCTeleOpControllerV4 extends OpMode {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor pixelBase;
    DcMotor pixelSlide; // from 0 to -5260
    PIDFArmController pidfArmController;
    double pixelArmToRadiansConstant = -1 / 50.9 * 4.5 * (Math.PI/180);

    public static double p = -1.3;
    public static double kg = -1;
    public static double i;
    public static double d;
    public static double kv = -1.5;
    public static double ka;

    DcMotor hookArm;
    Servo hookElbow;
    Servo fingerRight;
    Servo fingerLeft;
    Servo wristServo;
    Servo droneServo;

    DigitalChannel pixelSlideSwitch;

    boolean fingers;
    boolean oldXButton;
    boolean oldAButtonC2;
    boolean oldBButtonC2;
    boolean elbow;
    boolean oldCircle;
    boolean oldTriangle;

    int pixelPlacerState;
    //int oldPlacerState;
    int substate = -1;
    int pastSubstate = -2;

    ColorRangeSensor colorMan;

    double speedCoefficient = 1;

    private boolean firstTimeSlide = false;

    private double targetPosition;

    private int zeroOffset = 20;
     boolean isUp;

     private int downGrabTimer = 0;


    @Override
    public void init() {

        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        leftBack = hardwareMap.get(DcMotor.class, "backLeft");
        rightBack = hardwareMap.get(DcMotor.class, "backRight");

        //colorMan = hardwareMap.get(ColorRangeSensor.class, "colorSensor");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

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

        pidfArmController = new PIDFArmController(p,i,d,kv,ka,kg,0);
        //kg:-1
        //kv:-1.5
        //p:-1.2
        pidfArmController.launch(-15 *Math.PI/180,System.currentTimeMillis());

        //pixelArm.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(0, 0, 0, 0));

        hookArm = hardwareMap.get(DcMotorEx.class, "hookArm");
        hookArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hookArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hookElbow = hardwareMap.get(Servo.class, "hookElbow");

        hookElbow.setPosition(1);

    }

    @Override
    public void loop() {
        //leftFront.setPower(0.9);
        Controller1();
        Controller2();
        //telemetry();
    }

    public void Controller1() {
        driveMechanum();
        hangerControl();
        launchDrone();
    }

    public void Controller2() {

        fingerControl();
        controlPixelPlacer();
        wristControl();
        updateDownGrabTimer();
    }

    public void wristControl()
    {


//        if(gamepad2.dpad_down && pixelSlidePosition() < -1900 && !isUp)
//            wristServo.setPosition(0.87);
//        else
//            wristServo.setPosition(0.2);

        if(pixelPlacerState == 1)
        {
            wristServo.setPosition(0.74);
        }
        else if(pixelPlacerState == 2)
        {
            wristServo.setPosition(0);
        }
        else
        {
            wristServo.setPosition(0.1);
        }


    }

    public void launchDrone()
    {
        if(gamepad1.left_bumper)
            droneServo.setPosition(0);
        else
            droneServo.setPosition(0.5);

        telemetry.addData("Drone Servo", droneServo.getPosition());
    }

    public void updateDownGrabTimer()
    {
        downGrabTimer--;
        if(downGrabTimer == 0)
        {
            pixelPlacerState = 0;
            slideState = 0;
        }
    }

    public void controlPixelPlacer() {

        telemetry.addData("Current arm position: ", pixelBase.getCurrentPosition() * (pixelArmToRadiansConstant / Math.PI) * 180);

        //limit switch zero
        if(pixelSlideSwitch.getState())
        {
            telemetry.addLine("Zeroing");
            zeroOffset = pixelSlide.getCurrentPosition();
        }

        isUp = false;
        if (pixelPlacerState == 2) // up
        {
            isUp = true;
            //pixelArm.setTargetPosition(-568);
            //pixelArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //pixelArm.setPower(-0.6);

            targetPosition = 130 * Math.PI / 180;

            if(Math.abs(targetPosition - (pixelBase.getCurrentPosition() * pixelArmToRadiansConstant)) < 1.5)
            {
                telemetry.addLine("targeting good");
                setPixelSlide();
                runPixelSlide(1);
            }
            else
            {
                resetPixelSlide();
                runPixelSlide(15);
            }

//            if(fingers)
//            {
//                pixelPlacerState = 0;
//                slideState = 0;
//            }
            if (gamepad2.left_bumper && !fingers) {
                downGrabTimer = 22;
            }
        }
        else if (pixelPlacerState == 1) // down
        {
            //pixelArm.setTargetPosition(0);
            //pixelArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //pixelArm.setPower(0.6);

            targetPosition = -48 * Math.PI / 180;

            if(Math.abs(targetPosition -  (pixelBase.getCurrentPosition() * pixelArmToRadiansConstant)) < 1.5)
            {
                telemetry.addLine("targeting good");
                pixelSlide.setTargetPosition(-2880);
                pixelSlide.setPower(-1);
                runPixelSlide(2);
            }
            else
            {
                resetPixelSlide();
                runPixelSlide(10);
            }

            if (gamepad2.left_bumper && fingers) {
                downGrabTimer = 8;
            }
        }
        else // middle
        {
            //pixelArm.setTargetPosition(-30);
            //pixelArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ///pixelArm.setPower(-0.6);
            if(pixelSlidePosition() > -300)
            {
                targetPosition = -42 * Math.PI / 180;

                if (gamepad2.a != oldAButtonC2 && gamepad2.a)
                    pixelPlacerState = 1;
                else if (gamepad2.y != oldBButtonC2 && gamepad2.y)
                    pixelPlacerState = 2;
            }


            resetPixelSlide();
            runPixelSlide(0);
        }
        telemetry.addData("target: ", targetPosition / Math.PI * 180);
        telemetry.addData("error: ", targetPosition -  (pixelBase.getCurrentPosition() * pixelArmToRadiansConstant));
        //50.9 to 1 ; 105*Math.PI/180
        double correction = pidfArmController.updateArm(targetPosition, (pixelBase.getCurrentPosition() * pixelArmToRadiansConstant), 0,System.currentTimeMillis());
        telemetry.addData("correction: ", correction);

        pixelBase.setPower(correction);

        telemetry.addData("Pixel state: ", pixelPlacerState);

        //always keep at end
        oldAButtonC2 = gamepad2.a;
        oldBButtonC2 = gamepad2.y;

        //pixelArm.setPower(gamepad2.left_stick_y);

        telemetry.addData("Arm extention position", pixelSlidePosition());

        telemetry.addData("Slide if",firstTimeSlide);
    }

    private int slideState = 1;
    private final int slideMultiplyer = 900;
    private final int slideOffset = 300;

    private void setPixelSlide()
    {
        if(gamepad2.dpad_up)
        {
            slideState = 8;
            //tryReturn();
        }
        if(gamepad2.y)
        {
            slideState = 4;
            //tryReturn();
        }
        if(gamepad2.x)
        {
            slideState = 3;
            //tryReturn();
        }
        if(gamepad2.b)
        {
            slideState = 2;
            //tryReturn();
        }
        if(gamepad2.a)
        {
            slideState = 1;
            //tryReturn();
        }
        if(gamepad2.dpad_down)
        {
            slideState = 0;
            //tryReturn();
        }

        pixelSlide.setTargetPosition(Math.min(Math.max(-(slideState*slideMultiplyer + slideOffset),-5200),0));
        pixelSlide.setPower(-1);
        runPixelSlide(49656+slideState);
    }

    public void tryReturn()
    {
        if(!fingers) {
            pixelPlacerState = 0;
            slideState = 0;
        }
    }


    public int pixelSlidePosition()
    {
        //offset from limit switch
        return -zeroOffset + pixelSlide.getCurrentPosition();
    }

    public void resetPixelSlide()
    {
        pixelSlide.setTargetPosition(0);
        pixelSlide.setPower(-1);
        runPixelSlide(0);
    }

    public void runPixelSlide(int subStateSet)
    {
        substate = subStateSet;
        if(substate != pastSubstate)
        {
            firstTimeSlide = true;
        }
        pastSubstate = substate;

        if(firstTimeSlide) {
            pixelSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            firstTimeSlide = false;
        }
    }

    public void driveMechanum() {
        double leftBackPower;
        double rightBackPower;
        double leftFrontPower;
        double rightFrontPower;

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        updateSpeedCoefficient();

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

//        leftBackPower   = Math.cbrt(leftBackPower);
//        rightBackPower  = Math.cbrt(rightBackPower);
//        leftFrontPower  = Math.cbrt(leftFrontPower);
//        rightFrontPower = Math.cbrt(rightFrontPower);

        leftBackPower   = (leftBackPower * speedCoefficient);
        rightBackPower  = (rightBackPower * speedCoefficient);
        leftFrontPower  = (leftFrontPower * speedCoefficient);
        rightFrontPower = (rightFrontPower * speedCoefficient);

        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
    }

    public void telemetry() {
        telemetry.update();
    }

    public void updateSpeedCoefficient() {

        // Speed coefficient dpad up n down
//        if (gamepad1.dpad_up)
//        {
//            speedCoefficient = 1;
//        }
//        if (gamepad1.dpad_down)
//        {
//
//            speedCoefficient = 0.5;
//        }

        speedCoefficient = 1-(gamepad1.right_trigger*0.54);
    }

    private void fingerControl() {

        if(pixelPlacerState != 0)
        {
            //open ok
            //close ok

            boolean fingerToggle = gamepad2.left_bumper;
            if (fingerToggle && !oldXButton) {
                fingers = !fingers;
            }
            oldXButton = fingerToggle;

        }
        else
        {
            //auto close

            fingers = true;
        }

        if (fingers) {
            fingerRight.setPosition(1);
            fingerLeft.setPosition(0);
        } else {
            fingerRight.setPosition(0.65);
            fingerLeft.setPosition(0.35);
        }
    }

    private void hangerControl() {

        boolean elbowToggle = gamepad1.dpad_left;
        if (elbowToggle && !oldTriangle)
        {
            elbow = !elbow;
            if (elbow)
            {
                hookArm.setTargetPosition(-2400);
                hookArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hookArm.setPower(1);
                hookElbow.setPosition(0);
            }
            else
            {
                hookArm.setTargetPosition(0);
                hookArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hookArm.setPower(0.97);


            }
        }
        if(gamepad1.dpad_right) //panic reset
        {
            hookArm.setTargetPosition(0);
            hookArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hookArm.setPower(1);
            hookElbow.setPosition(1);
        }

        telemetry.addData("HookArm position [0-1]: ", -hookArm.getCurrentPosition() / 2400);
        oldTriangle = elbowToggle;
    }
}

