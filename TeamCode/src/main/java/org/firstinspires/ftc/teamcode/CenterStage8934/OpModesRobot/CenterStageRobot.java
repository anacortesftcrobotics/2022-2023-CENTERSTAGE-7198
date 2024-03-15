package org.firstinspires.ftc.teamcode.CenterStage8934.OpModesRobot;

import android.view.View;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ThreadPool;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CenterStage8934.OutdatedCode.OdoControllerAlfalfa;
import org.firstinspires.ftc.teamcode.kinematics.Pose2D;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;


import java.util.concurrent.ExecutorService;

public class CenterStageRobot {
    DigitalChannel bucketStop;
    Servo bucketServo, shoulderServo, intakeElbow, hookElbow, fingerLf, fingerRf, launcherServo;
    DcMotor frontLeft, frontRight, backLeft, backRight, viperSlide, hookArm;
    DcMotor encoderLeft, encoderRight, encoderCenter; // odometry wheels
    private ExecutorService threadExecuter;
    private LinearOpMode linearOpMode;
    public OdoControllerAlfalfa kaiOdo;
    private Runnable backgroundThread;

    NormalizedColorSensor colorSensor;
    NormalizedRGBA colors = new NormalizedRGBA();
    View relativeLayout;
    float gain = 10;
    public enum COLOR {
        RED,
        BLUE,
        WHITE,
        TILE
    }
    COLOR CURRENT_COLOR;
    double Kp = 1;
    double Ki = 0.1;
    double Kd = 0.01;

    PIDCoefficients coefficients = new PIDCoefficients(Kp,Ki,Kd);
    BasicPID controller = new BasicPID(coefficients);

    private boolean viperThreadLock = false;
    public CenterStageRobot(LinearOpMode linearOpMode, HardwareMap hardwareMap)
    {
        this.linearOpMode = linearOpMode;

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
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        colorSensor.setGain(gain);
        colors = colorSensor.getNormalizedColors();


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

        kaiOdo = new OdoControllerAlfalfa();
        kaiOdo.initializeHardware(hardwareMap);
        kaiOdo.resetEncoders();

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

    public void colorChanger()
    {
        if(colors.blue <= 0.1)
        {
            CURRENT_COLOR = COLOR.RED;
        }
        if(colors.blue >= 0.55)
        {
            CURRENT_COLOR = COLOR.BLUE;
        }
        if(colors.blue >= 0.9 & colors.green >= 0.9)
        {
            CURRENT_COLOR = COLOR.WHITE;
        }
        if(CURRENT_COLOR != COLOR.RED & CURRENT_COLOR != COLOR.BLUE & CURRENT_COLOR != COLOR.WHITE)
        {
            CURRENT_COLOR = COLOR.TILE;
        }

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

    public void placeAccurateBoard()
    {
        viperSlide.setTargetPosition(-475);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(-0.75);
        roboNap(600);
        bucketServo.setPosition(0.6);
        roboNap(300);
        bucketServo.setPosition(0.8);
    }

    public void viperRetract()
    {
        viperSlide.setTargetPosition(-10);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(0.75);
    }

    public void placeBoard()
    {
        viperSlide.setTargetPosition(-1000);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(-1);
        roboNap(750);
        bucketServo.setPosition(0.6);
        roboNap(300);
        bucketServo.setPosition(0.8);
        viperSlide.setTargetPosition(-10);
        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperSlide.setPower(1);
    }

    public void grabInitialPixelsOdo()
    {
        // FIXME: generalize to a teleop automation
        // FIXME: replace with autograb that uses sensors
        fingerLf.setPosition(0);
        fingerLf.setPosition(1);
        roboNap(1700);

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
     */
    public void moveRobotPosition(double deltaY_cm, double deltaX_cm, double deltaA_deg, Telemetry telem) {
        double taRad = (deltaA_deg * Math.PI / 180);
        //kaiOdo.resetEncoders();
        //Pose2D targetLocation = new Pose2D(new Vector2D(kaiOdo.getX() + deltaX_cm, kaiOdo.getY() + deltaY_cm), taRad + kaiOdo.getHeadingRad());
        Pose2D targetLocation = new Pose2D(deltaX_cm,deltaY_cm,taRad);

        double timer = 5;

        if(linearOpMode != null)
            while(timer >= 0 && linearOpMode.opModeIsActive()) //in cm
            {

                kaiOdo.update();

                telem.addData("Target: ", targetLocation.getX() + ", " + targetLocation.getY() + ", " + targetLocation.getHeadingRad());
                telem.addData("Pose out: ", Math.round(kaiOdo.getX()) + " " + Math.round(kaiOdo.getY()) + " " + Math.round(kaiOdo.getHeadingRad()));
                telem.addData("Pose out: ", kaiOdo.getX() + " " + kaiOdo.getY() + " " + kaiOdo.getHeadingDeg());

                double absoluteXToPosition = targetLocation.getX() - kaiOdo.getX();
                double absoluteYToPosition = targetLocation.getY() - kaiOdo.getY();

                double absoluteAngleToPosition = Math.atan2(absoluteYToPosition, absoluteXToPosition);
                double distanceToPosition = Math.hypot(absoluteXToPosition, absoluteYToPosition);

                double relativeAngleToPosition = angleWrap(absoluteAngleToPosition - kaiOdo.getHeadingRad());
                //telem.addData("relativeAngleToPosition: ", relativeAngleToPosition);

                double relativeXToPosition = distanceToPosition * Math.cos(relativeAngleToPosition);
                double relativeYToPosition = distanceToPosition * Math.sin(relativeAngleToPosition);

                double powerX = relativeXToPosition / (Math.abs(relativeXToPosition) + Math.abs(relativeYToPosition));
                double powerY = relativeYToPosition / (Math.abs(relativeXToPosition) + Math.abs(relativeYToPosition));
                double powerTurn = angleWrap(-kaiOdo.getHeadingRad() + targetLocation.getHeadingRad()) / Math.PI;

                double multiplier = 1;
                double d = getDistance(targetLocation.getX(),targetLocation.getY(), kaiOdo.getX(), kaiOdo.getY());
                if(d < 80) //cm
                {
                    multiplier = Math.pow(d/80,2);
                    multiplier = Math.max(multiplier,0.3);
                }

                //telem.addData("distance", d);
                //telem.addData("multiplier", multiplier);
                telem.addLine("Y: " + (powerY * multiplier) + ", X: " + (powerX * multiplier));
                telem.addData("powerTurn:", powerTurn);

                //multiplier *= 0.5; //temp for test
                //mecanumX(-powerY * multiplier,powerX * multiplier,-powerTurn);
                mecanumX(powerY * multiplier,-powerX * multiplier,-powerTurn*1.5*0);

                //telem.update();

                if(getDistance(targetLocation.getX(),targetLocation.getY(), kaiOdo.getX(), kaiOdo.getY()) < 3 && Math.abs(powerTurn) < 0.3)
                    timer--;

                telem.update();
            }

        mecanumX(0,0,0);

    }

    public void moveRobotPosition_IN(double deltaY_in, double deltaX_in, double deltaA_deg,Telemetry telem)
    {
        moveRobotPosition(deltaY_in*2.54,deltaX_in*2.54,deltaA_deg,telem);
    }

    public double getDistance(double tx, double ty, double ox, double oy)
    {
        return Math.sqrt((tx - ox)*(tx - ox) + (ty - oy)*(ty - oy));
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
