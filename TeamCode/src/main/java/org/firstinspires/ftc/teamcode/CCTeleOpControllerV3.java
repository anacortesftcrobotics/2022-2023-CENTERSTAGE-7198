package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.kaicode.PIDFArmController;

@TeleOp
@Config
public class CCTeleOpControllerV3 extends OpMode {
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

        pidfArmController = new PIDFArmController(p,i,d,kv,ka,kg,0);
        //kg:-1
        //kv:-1.5
        //p:-1.2
        pidfArmController.launch(-28 *Math.PI/180,System.currentTimeMillis());

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
    }

    public void Controller2() {

        controlPixelPlacer();
        fingerControl();
        wristControl();
        launchDrone();
    }

    public void wristControl()
    {
        if(gamepad2.dpad_down)
            wristServo.setPosition(0.86);
        else
            wristServo.setPosition(0.4);
    }

    public void launchDrone()
    {
        if(gamepad2.left_stick_button)
            droneServo.setPosition(0);
        else
            droneServo.setPosition(1);
    }

    public void controlPixelPlacer() {

        telemetry.addData("Current arm position: ", pixelBase.getCurrentPosition() * (pixelArmToRadiansConstant / Math.PI) * 180);

        if (pixelPlacerState == 2) // up
        {
            //pixelArm.setTargetPosition(-568);
            //pixelArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //pixelArm.setPower(-0.6);

            targetPosition = 160 * Math.PI / 180;

            if(Math.abs(targetPosition - (pixelBase.getCurrentPosition() * pixelArmToRadiansConstant)) < 1.5)
            {
                telemetry.addLine("targeting good");
                pixelSlide.setTargetPosition(-5200);
                pixelSlide.setPower(-1);
                runPixelSlide(1);
            }
            else
            {
                resetPixelSlide();
                runPixelSlide(15);
            }
        }
        else if (pixelPlacerState == 1) // down
        {
            //pixelArm.setTargetPosition(0);
            //pixelArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //pixelArm.setPower(0.6);

            targetPosition = -29 * Math.PI / 180;

            if(Math.abs(targetPosition -  (pixelBase.getCurrentPosition() * pixelArmToRadiansConstant)) < 1.5)
            {
                telemetry.addLine("targeting good");
                pixelSlide.setTargetPosition(-5200);
                pixelSlide.setPower(-1);
                runPixelSlide(2);
            }
            else
            {
                resetPixelSlide();
                runPixelSlide(10);
            }
        }
        else // middle
        {
            //pixelArm.setTargetPosition(-30);
            //pixelArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ///pixelArm.setPower(-0.6);
            if(pixelSlide.getCurrentPosition() > -300)
            {
                targetPosition = -25 * Math.PI / 180;

                if (gamepad2.a != oldAButtonC2 && gamepad2.a)
                    pixelPlacerState = 1;
                else if (gamepad2.b != oldBButtonC2 && gamepad2.b)
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


        if (gamepad2.left_bumper)
            pixelPlacerState = 0;

        telemetry.addData("Pixel state: ", pixelPlacerState);

        //always keep at end
        oldAButtonC2 = gamepad2.a;
        oldBButtonC2 = gamepad2.b;

        //pixelArm.setPower(gamepad2.left_stick_y);

        telemetry.addData("Arm extention position", pixelSlide.getCurrentPosition());

        telemetry.addData("Slide if",firstTimeSlide);
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

        x = deadZone(x,0.02);
        y = deadZone(y,0.02);
        rx = deadZone(rx,0.02);

        //rx += poleCenter();

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        leftBackPower = (y - x + rx) / denominator;
        rightBackPower = (y + x - rx) / denominator;
        leftFrontPower = (y + x + rx) / denominator;
        rightFrontPower = (y - x - rx) / denominator;

        leftBackPower = Math.cbrt(leftBackPower);
        rightBackPower = Math.cbrt(rightBackPower);
        leftFrontPower = Math.cbrt(leftFrontPower);
        rightFrontPower = Math.cbrt(rightFrontPower);

        leftBackPower = exponentialRemapAnalog(leftBackPower);
        rightBackPower  = exponentialRemapAnalog(rightBackPower);
        leftFrontPower  = exponentialRemapAnalog(leftFrontPower);
        rightFrontPower = exponentialRemapAnalog(rightFrontPower);

        leftBackPower = (leftBackPower * speedCoefficient);
        rightBackPower = (rightBackPower * speedCoefficient);
        leftFrontPower = (leftFrontPower * speedCoefficient);
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
        if (gamepad1.dpad_up)
        {
            speedCoefficient = 1;
        }
        if (gamepad1.dpad_down)
        {

            speedCoefficient = 0.5;
        }
    }

    //remaps input so small values near 0 are 0 and values above a threshold are valued.
    //The function will reach 1 when double input is 1
    //
    //double a is in the range [0 - Infinity)
    //
    //calcualated in desmos
    public double deadZone(double input,double a) {

        a = Math.max(a, 0);
        return Math.max(x(1+a),a)+Math.min(x(1+a),-a);
    }

    //the following function remaps double x to a expnetial equation keeping its sign.
    //
    //calculated in desmos
    public double exponentialRemapAnalog(double x) {
        return Math.min(Math.pow(Math.max(x,0),2),1) + Math.pow(Math.max(-Math.min(x,0),2),-1);
    }

    private void fingerControl() {

        if(pixelSlide.getCurrentPosition() < -1200)
        {
            //open ok
            //close ok

            boolean fingerToggle = gamepad2.x;
            if (fingerToggle && !oldXButton) {
                fingers = !fingers;
                if (fingers) {
                    fingerRight.setPosition(1);
                    fingerLeft.setPosition(0);
                } else {
                    fingerRight.setPosition(0.6);
                    fingerLeft.setPosition(0.54);
                }
            }
            oldXButton = fingerToggle;

        }
        else
        {
            //auto close

            fingerRight.setPosition(1);
            fingerLeft.setPosition(0);
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
            hookElbow.setPosition(0.97);
        }

        telemetry.addData("HookArm position [0-1]: ", -hookArm.getCurrentPosition() / 2400);
        oldTriangle = elbowToggle;
    }
}

