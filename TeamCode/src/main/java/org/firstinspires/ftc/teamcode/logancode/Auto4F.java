package org.firstinspires.ftc.teamcode.logancode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Team7198PropProcessor;
import org.firstinspires.ftc.teamcode.kaicode.Odo1;
import org.firstinspires.ftc.teamcode.kaicode.PIDFArmController;
import org.firstinspires.ftc.teamcode.kaicode.PIDFController;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Auto4F", group = "Autos")
public class Auto4F extends LinearOpMode
{

    //Camera Processing
    VisionPortal visionPortal;
    Team7198PropProcessor visProcessor;
    private int visionData = -1;

    //Motors
    private DcMotor leftBack, leftFront, rightBack, rightFront;

    //Odometry
    private DcMotor encoderRight, encoderLeft, encoderBack;
    DcMotor pixelBase;
    DcMotor pixelSlide; // from 0 to -5260
    Servo fingerRight;
    Servo fingerLeft;
    Servo wristServo;
    private Odo1 kaiOdo;

    PIDFArmController pidfArmController;
    double pixelArmToRadiansConstant = -1 / 50.9 * 4.5 * (Math.PI/180);

    public static double p = -1.3;
    public static double kg = -1;
    public static double i;
    public static double d;
    public static double kv = -1.5;
    public static double ka;

    private double tempLastDegrees = 0;
    private double pastTime = 0;

    private double greatestVelocity = 0;

    private PIDFController Xpidf = new PIDFController(0.001,0,0,0.1,0.1);
    //0.0045 //0 //0 //0.46 // 0.4
    private PIDFController Ypidf = new PIDFController(0.001,0,0,0.34,0.15);
    private PIDFController Rpidf = new PIDFController(0.01,0,0,14,0.8);
    //0.9 : 0.000008 : 0

    private final double PATH_TOLERANCE = 4.5;

    public void runOpMode() {
        mapHardware();
        resetDriveEncoder();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //telemetry.addData("Velocity: ", 0);
        //telemetry.addData("Target Velocity: ", 0);

        kaiOdo = new Odo1(36.2, 25.8, 4.8, 2000);

        pidfArmController = new PIDFArmController(p, i, d, kv, ka, kg, 0);

        visProcessor = new Team7198PropProcessor(true, this);

        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), visProcessor);

        visionPortal.resumeLiveView();

        //FtcDashboard.getInstance().startCameraStream(visProcessor, 0);

        Xpidf.reset();
        Ypidf.reset();
        Rpidf.reset();
        Xpidf.launch(0, System.currentTimeMillis());
        Ypidf.launch(0, System.currentTimeMillis());
        Rpidf.launch(0, System.currentTimeMillis());

        int fileId = hardwareMap.appContext.getResources().getIdentifier("short_path_l", "raw", hardwareMap.appContext.getPackageName());
        Path shortPathL = new Path(hardwareMap.appContext.getResources().openRawResource(fileId), telemetry);
        int fileId1 = hardwareMap.appContext.getResources().getIdentifier("long_path", "raw", hardwareMap.appContext.getPackageName());
        Path longPath = new Path(hardwareMap.appContext.getResources().openRawResource(fileId1), telemetry);
        int fileId2 = hardwareMap.appContext.getResources().getIdentifier("short_path_r", "raw", hardwareMap.appContext.getPackageName());
        Path shortPathR = new Path(hardwareMap.appContext.getResources().openRawResource(fileId2), telemetry);
        int fileId3 = hardwareMap.appContext.getResources().getIdentifier("final_path", "raw", hardwareMap.appContext.getPackageName());
        Path finalPath = new Path(hardwareMap.appContext.getResources().openRawResource(fileId3), telemetry);

        fingerRight.setPosition(0);
        fingerLeft.setPosition(1);
        wristServo.setPosition(0);

        waitForStart();

        //scan with camera
        int TeamPropLocation = -1;
        while (TeamPropLocation == -1 && opModeIsActive())
        {
            TeamPropLocation = visProcessor.data;
            telemetry.addData("Data: ", visProcessor.data);
            telemetry.addData("Camera FPS: ", visionPortal.getFps());
            telemetry.addData("Camera State: ", visionPortal.getCameraState());
            telemetry.addData("System Time: ", System.currentTimeMillis() / 1000);
            telemetry.update();
        }
        telemetry.addData("Vision date: ", TeamPropLocation);
        telemetry.update();
        //go to tape
        if(TeamPropLocation == 1) {

            //go forward
            followPath(longPath, "Long Path");

            //place purple
            placePurple();
        }
        else
        {
            //go forward but less

            if(TeamPropLocation == 2)
            {
                //place right
                followPath(shortPathR, "Right path");

                placePurple();
            }
            else
            {
                //place left
                followPath(shortPathL, "Left path");

                placePurple();
            }
        }

        //go to known point
        PathMarker pathPosition = new PathMarker(0,0,0,0,0,0); //find good x and y or 0 ok

        while(!checkIfNearPathMarker(pathPosition, new PathMarker(kaiOdo.getX(), kaiOdo.getY(), kaiOdo.getHRad(), kaiOdo.getDeltaX(), kaiOdo.getDeltaY(), kaiOdo.getDeltaHRad())) && opModeIsActive())
        {
            PathMarker p = new PathMarker(kaiOdo.getX(), kaiOdo.getY(), kaiOdo.getHRad(), kaiOdo.getDeltaX(), kaiOdo.getDeltaY(), kaiOdo.getDeltaHRad());
            telemetry.addData("Position", p);
            telemetry.addData("Distance: ", pathPosition.distance(p));


            traverseToPosition(pathPosition, new PathMarker(kaiOdo.getX(), kaiOdo.getY(), kaiOdo.getHRad(), kaiOdo.getDeltaX(), kaiOdo.getDeltaY(), kaiOdo.getDeltaHRad()));
            telemetry.update();
        }
        PerformLocalMovement(0,0,0);

        //go to parking location
        followPath(finalPath, "final path");

        //end of linear program //end of linear program //end of linear program //end of linear program
        visionPortal.close();
    }

    public void placePurple()
    {
        boolean placed = false;
        boolean backUp = false;
        double targetPosition = -28 * Math.PI / 180;
        int i = -80;
        while (!placed && opModeIsActive())
        {
            double correction = pidfArmController.updateArm(targetPosition, (pixelBase.getCurrentPosition() * pixelArmToRadiansConstant), 0,System.currentTimeMillis());
            telemetry.addData("correction: ", correction);
            pixelBase.setPower(correction);

            if(!backUp) {
                pixelSlide.setTargetPosition(-5200);
            }
            else {
                pixelSlide.setTargetPosition(0);
                wristServo.setPosition(0.4);
                if(pixelSlide.getCurrentPosition() > -200)
                {
                    placed = true;
                }
            }
            if (i == 0) {
                fingerLeft.setPosition(1);
                pixelSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                pixelSlide.setPower(-0.6);
                wristServo.setPosition(0.4);
            }

            if(pixelSlide.getCurrentPosition() < -1000)
                wristServo.setPosition(0.86);

            if(pixelSlide.getCurrentPosition() < -5000)
            {
                fingerLeft.setPosition(0.54);
                i = -80;
                backUp = true;
            }


            i++;
        }
        pixelBase.setPower(0);
    }

    public void followPath(Path path, String name)
    {
        int currentPathIndex = 0;
        PathMarker pathPosition = path.getPosition(currentPathIndex);

        while (opModeIsActive() && pathPosition != null) {
            kaiOdo.setEncoderPos(-encoderLeft.getCurrentPosition(),
                    encoderRight.getCurrentPosition(),
                    encoderBack.getCurrentPosition());

            pathPosition = path.getPosition(currentPathIndex);

            if (pathPosition != null) {


                if (traverseToPosition(pathPosition, new PathMarker(kaiOdo.getX(), kaiOdo.getY(), kaiOdo.getHRad(), kaiOdo.getDeltaX(), kaiOdo.getDeltaY(), kaiOdo.getDeltaHRad()))) {
                    currentPathIndex++;
                    if (currentPathIndex < path.length() - 1)
                        while (checkIfNearPathMarker(path.getPosition(currentPathIndex), new PathMarker(kaiOdo.getX(), kaiOdo.getY(), kaiOdo.getHRad(), kaiOdo.getDeltaX(), kaiOdo.getDeltaY(), kaiOdo.getDeltaHRad()))) {
                            currentPathIndex++;
                            if (currentPathIndex >= path.length())
                                break;
                        }

                    //telemetry.addData("Velocity: ", kaiOdo.getDeltaHDeg());
                    //telemetry.addData("Target Velocity: ", pathPosition.getVr());
                }
                telemetry.addData("Path Name: ", name);
                telemetry.addLine("Path index: " + currentPathIndex + "/" + path.length());
                telemetry.update();
            }
        }
        PerformLocalMovement(0,0,0);
    }

    public boolean traverseToPosition(PathMarker target, PathMarker currentPosition)
    {
        double rotation = Rpidf.update(target.getR() * Math.PI/180,currentPosition.getR(), target.getVr() * Math.PI/180,System.currentTimeMillis());
        double x = Xpidf.update(target.getX(),currentPosition.getX(), target.getVx(), System.currentTimeMillis());
        double y = Ypidf.update(target.getY(), currentPosition.getY(), target.getVy(), System.currentTimeMillis());
        //double y = 0; double x = 0;
        performGlobalMovement(x,y,rotation);

        telemetry.addLine("IsInBetween output: " + isInBetween(currentPosition.getR(), currentPosition.getR() - currentPosition.getVr(), target.getR() * Math.PI/180, PATH_TOLERANCE));

    return checkIfNearPathMarker(target, currentPosition);
        //return currentPosition.distance(target) + Math.abs(currentRadRot - target.getR());
    }

    public boolean checkIfNearPathMarker(PathMarker target, PathMarker currentPosition)
    {
        if(target != null) {
            boolean clear = isInBetween(currentPosition.getX(), currentPosition.getX() - currentPosition.getVx(), target.getX(), PATH_TOLERANCE)
                    && isInBetween(currentPosition.getY(), currentPosition.getY() - currentPosition.getVy(), target.getY(), PATH_TOLERANCE)
                    && isInBetween(currentPosition.getR(), currentPosition.getR() - currentPosition.getVr(), target.getR() * Math.PI/180, PATH_TOLERANCE)
                    ;
            return clear;
        }
        return true;
    }

    public boolean isInBetween(double a, double b, double target, double tolerance)
    {
        return (target >= a-tolerance && target <= b+tolerance) || (target >= b-tolerance && target <= a+tolerance);
    }

    public void performGlobalMovement(double x, double y, double rx)
    {
        double xl, yl;
        yl = (x * Math.cos(kaiOdo.getHRad())) + (y * Math.sin(kaiOdo.getHRad()));
        xl = (x * Math.sin(kaiOdo.getHRad())) - (y * Math.cos(kaiOdo.getHRad()));

        PerformLocalMovement(xl,yl,rx);
    }

    public void PerformLocalMovement(double x, double y, double rx)
    {
        double leftBackPower;
        double rightBackPower;
        double leftFrontPower;
        double rightFrontPower;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        leftBackPower = (y - x + rx) / denominator;
        rightBackPower = (y + x - rx) / denominator;
        leftFrontPower = (y + x + rx) / denominator;
        rightFrontPower = (y - x - rx) / denominator;

        leftBackPower = Math.cbrt(leftBackPower);
        rightBackPower = Math.cbrt(rightBackPower);
        leftFrontPower = Math.cbrt(leftFrontPower);
        rightFrontPower = Math.cbrt(rightFrontPower);

        leftBack.setPower(leftBackPower * 0.5);
        rightBack.setPower(rightBackPower * 0.5);
        leftFront.setPower(leftFrontPower * 0.5);
        rightFront.setPower(rightFrontPower * 0.5);

    }

    public void odoTelemetry()
    {
        telemetry.addLine("Internal Position:");
        telemetry.addData("(X, Y, Degrees)", Math.round(kaiOdo.getX() *10)/10d + " : " + Math.round(kaiOdo.getY() *10)/10d + " : " + Math.round(kaiOdo.getHDeg() *10)/10d);
        telemetry.update();
    }

    public void resetDriveEncoder()
    {
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void mapHardware()
    {
        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        leftBack = hardwareMap.get(DcMotor.class, "backLeft");
        rightBack = hardwareMap.get(DcMotor.class, "backRight");

        pixelBase = hardwareMap.get(DcMotor.class, "pixelBase");
        pixelBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pixelBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pixelSlide = hardwareMap.get(DcMotorEx.class, "pixelSlide");
        pixelSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pixelSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fingerRight = hardwareMap.get(Servo.class, "servoB");
        fingerLeft = hardwareMap.get(Servo.class, "servoA");

        wristServo = hardwareMap.get(Servo.class, "wristServo");

        encoderBack = rightFront;
        encoderLeft = leftBack;
        encoderRight = leftFront;

        //colorMan = hardwareMap.get(ColorRangeSensor.class, "colorSensor");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void recieveVisionInfo(int x)
    {

        visionData = x;
        //telemetry.update();
    }
}
