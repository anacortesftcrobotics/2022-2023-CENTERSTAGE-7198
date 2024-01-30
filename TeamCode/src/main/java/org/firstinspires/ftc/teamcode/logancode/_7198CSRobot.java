package org.firstinspires.ftc.teamcode.logancode;

import com.qualcomm.hardware.lynx.LynxEmbeddedBNO055IMUNew;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ThreadPool;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.kaicode.PIDFArmController;
import org.firstinspires.ftc.teamcode.kaicode.PIDFController;
import org.firstinspires.ftc.teamcode.kinematics.Pose2D;
import org.firstinspires.ftc.teamcode.kinematics.Vector2D;
import org.firstinspires.ftc.teamcode.odometry._7198_OdoController;

import java.util.concurrent.ExecutorService;

public class _7198CSRobot {
    DigitalChannel bucketStop;
    Servo fingerRight, fingerLeft, wristServo, droneServo;
    DigitalChannel pixelSlideSwitch;
    DcMotor frontLeft, frontRight, backLeft, backRight, pixelBase, pixelSlide;
    private _7198_OdoController kaiOdo;
    PIDFArmController pidfArmController;
    IMU imu;
    PoseTracker imuAngleTracker;
    PIDFController rotationPIDF = new PIDFController(0.0017,0,0.00004,0,0);
    double pixelArmToRadiansConstant = -1 / 50.9 * 4.5 * (Math.PI/180);

    public static double p = -1.3;
    public static double kg = -1;
    public static double i;
    public static double d;
    public static double kv = -1.5;
    public static double ka;
    private ExecutorService threadExecuter;
    private Runnable backgroundThread;
    private LinearOpMode linearOpMode;
    private boolean viperThreadLock = false;
    public _7198CSRobot(HardwareMap hardwareMap, LinearOpMode linearOpMode)
    {
        this.linearOpMode = linearOpMode;

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

        imu = hardwareMap.get(IMU.class,"imu");

        IMU.Parameters myIMUparameters;

         myIMUparameters = new IMU.Parameters(
              new RevHubOrientationOnRobot(
                   RevHubOrientationOnRobot.LogoFacingDirection.UP,
                   RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
              )
         );

         imu.initialize(myIMUparameters);
         imu.resetYaw();
         imuAngleTracker = new PoseTracker(imu);
         imuAngleTracker.resetAngle();

           // Get Robot Orientation

        //reverses direction of wheels (left)
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        pidfArmController = new PIDFArmController(p,i,d,kv,ka,kg,0);
        //kg:-1
        //kv:-1.5
        //p:-1.2
        pidfArmController.launch(-15 *Math.PI/180,System.currentTimeMillis());

        kaiOdo = new _7198_OdoController();
        kaiOdo.initializeHardware(hardwareMap);

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

    public Orientation getMyRobotOrientation()
    {
        return imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
    }

    /**
     * Rotates the robot by a Delta angle using the IMU.
     * Can take in a X and Y value. Poorly tested and unrecommended.
     * IMU Powered!
     *
     * @param forward
     * @param sideways
     * @param angle
     * @param armAngle
     * @param telemetry
     */
    public void setRobotRotation(double forward, double sideways, double angle, double armAngle, Telemetry telemetry)
    {
        int timer = 10;
        while(timer > 0 && linearOpMode.opModeIsActive())
        {
            telemetry.addData("IMU angle: ", imuAngleTracker.getAngle());
            telemetry.addData("TAR angle: ", angle);
            //telemetry.addData("IMU direct report", imu.getRobotYawPitchRollAngles());
            double correction = rotationPIDF.update(angle, imuAngleTracker.getAngle(), System.currentTimeMillis());
            telemetry.addData("Correction: ",correction);

            mecanumX(forward, sideways, correction);
            kaiOdo.update();
            SetArmAngle(armAngle);

            telemetry.update();

            if(Math.abs(angle - imuAngleTracker.getAngle()) < 2)
            {
                timer--;
            }
        }
        mecanumX(0,0,0);
    }

    /**
     * Wraps the able so it is always in the range [-180, 180].
     *
     * @param angle Angle to be wrapped, in radians.
     * @return The wrapped angle, in radians.
     */
    public static double angleWrap(double angle) {
        if (angle > 0)
            return ((angle + Math.PI) % (Math.PI * 2)) - Math.PI;
        else
            return ((angle - Math.PI) % (Math.PI * 2)) + Math.PI;
    }

    /**
     * Takes the robot's current position and rotation and calculates the motor powers for the robot to move to the target position and rotation.
     * This will loop until the robot has reached its target. WARNING: Do not call this function while moving.
     * Odo Powered!
     *
     * @param deltaY_cm       Delta delta movement in Y
     * @param deltaX_cm       Delta delta movement in X
     * @param deltaA_deg      Delta rotation (Degrees)
     * @param armAngle Target angle of the arm
     */
    public void moveRobotPosition(double deltaY_cm, double deltaX_cm, double deltaA_deg, double armAngle, Telemetry telem) {
        double taRad = (deltaA_deg * Math.PI / 180);
        //kaiOdo.resetEncoders();
        Pose2D targetLocation = new Pose2D(new Vector2D(kaiOdo.getX() + deltaX_cm, kaiOdo.getY() + deltaY_cm), taRad + kaiOdo.getHeadingRad());

        while(getDistance(targetLocation.getX(),targetLocation.getY(), kaiOdo.getX(), kaiOdo.getY()) > 0.75 && linearOpMode.opModeIsActive()) //in cm
        {

            kaiOdo.update();

            telem.addData("Target: ", targetLocation.getX() + ", " + targetLocation.getY() + ", " + targetLocation.getHeadingRad());
            telem.addData("Pose out: ", Math.round(kaiOdo.getX()) + " " + Math.round(kaiOdo.getY()) + " " + Math.round(kaiOdo.getHeadingRad()));
            //telem.addData("Pose out: ", kaiOdo.getX() + " " + kaiOdo.getY() + " " + kaiOdo.getHeadingDeg());


            double absoluteXToPosition = targetLocation.getX() - kaiOdo.getX();
            double absoluteYToPosition = targetLocation.getY() - kaiOdo.getY();

            double absoluteAngleToPosition = Math.atan2(absoluteYToPosition, absoluteXToPosition);
            double distanceToPosition = Math.hypot(absoluteXToPosition, absoluteYToPosition);

            double relativeAngleToPosition = angleWrap(absoluteAngleToPosition + kaiOdo.getHeadingRad());
            telem.addData("relativeAngleToPosition: ", relativeAngleToPosition);

            double relativeXToPosition = distanceToPosition * Math.cos(relativeAngleToPosition);
            double relativeYToPosition = distanceToPosition * Math.sin(relativeAngleToPosition);

            double powerX = relativeXToPosition / (Math.abs(relativeXToPosition) + Math.abs(relativeYToPosition));
            double powerY = relativeYToPosition / (Math.abs(relativeXToPosition) + Math.abs(relativeYToPosition));
            double powerTurn = angleWrap(kaiOdo.getHeadingRad() + targetLocation.getHeadingRad()) / Math.PI;

            double multiplier = 1;
            double d = getDistance(targetLocation.getX(),targetLocation.getY(), kaiOdo.getX(), kaiOdo.getY());
            if(d < 119) //cm
            {
                multiplier = d/119;
                multiplier = Math.max(multiplier,0.001);
            }

            telem.addData("distance", d);
            telem.addData("multiplier", multiplier);
            telem.addLine("Y: " + (powerY * multiplier) + ", X: " + (powerX * multiplier));
            //telem.addData("powerTurn:", powerTurn);

            multiplier = multiplier / 4;
            mecanumX(-powerY * multiplier,powerX * multiplier,-powerTurn);

            SetArmAngle(armAngle);
            telem.update();
        }

        mecanumX(0,0,0);
    }

    public double getDistance(double tx, double ty, double ox, double oy)
    {
        return Math.sqrt((tx - ox)*(tx - ox) + (ty - oy)*(ty - oy));
    }

    //TODO: this is just a meme now. don't delete it. its statements are invalid
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

    public void SetArmAngle(double targetPosition) {
        targetPosition = targetPosition*Math.PI/180;
        //telemetry.addData("target: ", targetPosition / Math.PI * 180);
        //telemetry.addData("error: ", targetPosition -  (pixelBase.getCurrentPosition() * pixelArmToRadiansConstant));
        //50.9 to 1 ; 105*Math.PI/180
        double correction = pidfArmController.updateArm(targetPosition, (pixelBase.getCurrentPosition() * pixelArmToRadiansConstant), 0,System.currentTimeMillis());
        //telemetry.addData("correction: ", correction);

        pixelBase.setPower(correction);
    }

    public void LimpArm()
    {
        pixelBase.setPower(0);
    }

    // used in Auto to intake the pixels
    public void grabInitialPixels(double armAngle)
    {
        // TODO: generalize to a teleop automation
        // TODO: replace with autograb that uses sensors

        //open fingers
        //fingerRight.setPosition(0.6);
        //fingerLeft.setPosition(0.54);

        //close fingers
        fingerRight.setPosition(1);
        fingerLeft.setPosition(0);
        robotArmNap(1500, armAngle);

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
    public void halt(double armAngle)
    {
        mecanumX(0, 0, 0);
        robotArmNap(50,armAngle);
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
    public void setIntakeToBatteringRam(double armAngle)
    {
//        intakeElbow.setPosition(1);
//        shoulderServo.setPosition(0.32);

        robotArmNap(100,armAngle);
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

        //setIntakeToBatteringRam();

        /*
        shoulderServo.setPosition(.6);
        intakeElbow.setPosition(.1);
        roboNap(500);
        viperSlide.setPower(.3);
*/
    }
    public void intakeDownGrabPixelsComeUp()
    {
//        //intakeElbow.setPosition(1);
//        roboNap(200);
//        //shoulderServo.setPosition(0.1);
//        robotArmNap(300);
//        fingerRight.setPosition(0);
//        fingerLeft.setPosition(1);
//        robotArmNap(700);
//        setIntakeToBatteringRam();
    }

    // wrapper around mecanumX and roboNap, because commonly those are called together

    /**
     * This method is outdated and drives blindly and variably due to battery.
     * @param forwards
     * @param sideways
     * @param rotate
     * @param sleepMillis
     * @param armAngle
     */
    @Deprecated
    public void drive(double forwards, double sideways, double rotate, int sleepMillis, double armAngle)
    {
        mecanumX(forwards, sideways, -rotate);
        if (sleepMillis > 0) {
            robotArmNap(sleepMillis,armAngle);
        }
    }
//
//    public void drive(double forwards, double sideways, double rotate, int sleepMillis, double armAngle)
//    {
//        mecanumX(forwards, sideways, -rotate);
//
//        long StartTime = System.currentTimeMillis();
//        long currentTime = System.currentTimeMillis();
//
//        if (sleepMillis > 0) {
//            while (StartTime - currentTime < sleepMillis)
//            {
//                SetArmAngle(armAngle);
//                currentTime = System.currentTimeMillis();
//            }
//        }
//    }

    /**
     * This function in general should be used only by low level robot commands or vision location commands
     *
     * @param y
     * @param x
     * @param rx
     */
    public void mecanumX(double y,double x, double rx) {
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

        double leftBackPower;
        double rightBackPower;
        double leftFrontPower;
        double rightFrontPower;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        leftBackPower = (y - x + rx) / denominator;
        rightBackPower = (y + x + rx) / denominator;
        leftFrontPower = (y + x - rx) / denominator;
        rightFrontPower = (y - x - rx) / denominator;

        leftBackPower   = Math.cbrt(leftBackPower);
        rightBackPower  = Math.cbrt(rightBackPower);
        leftFrontPower  = Math.cbrt(leftFrontPower);
        rightFrontPower = Math.cbrt(rightFrontPower);

        frontLeft.setPower(leftBackPower);
        backLeft.setPower(rightBackPower);
        frontRight.setPower(leftFrontPower);
        backRight.setPower(rightFrontPower);
    }
    public void robotArmNap(int millis,double armAngle)
    {
        long StartTime = System.currentTimeMillis();

        if (millis > 0) {
            while (System.currentTimeMillis() - StartTime  < millis && linearOpMode.opModeIsActive())
            {
                SetArmAngle(armAngle);
                try {
                    Thread.sleep(1);                   
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }
        }
    }

    public void closeRightFinger(double ArmAngle)
    {
        fingerLeft.setPosition(1);
        robotArmNap(1000,ArmAngle);
    }

}

