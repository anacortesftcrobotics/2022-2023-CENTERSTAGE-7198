package org.firstinspires.ftc.teamcode.logancode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Team7198PropProcessor;
import org.firstinspires.ftc.teamcode.kaicode.Odo1;
import org.firstinspires.ftc.teamcode.kaicode.PIDFController;
import org.firstinspires.ftc.teamcode.kinematics.PoseVelocity2D;
import org.firstinspires.ftc.teamcode.odometry._7198_OdoController;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Odo6PathFollower_Logan", group = "Autos")
public class Odo6PathFollower  extends LinearOpMode
{

    //Camera Processing
    VisionPortal visionPortal;
    Team7198PropProcessor visProcessor;
    private int visionData = 0;

    //Motors
    private DcMotor leftBack, leftFront, rightBack, rightFront;

    //Odometry
    private DcMotor encoderRight, encoderLeft, encoderBack;
    private _7198_OdoController kaiOdo;


    private double tempLastDegrees = 0;
    private double pastTime = 0;

    private double greatestVelocity = 0;

    private PIDFController Xpidf = new PIDFController(0.6,0,0,0.34,0.15);
    //0.0045 //0 //0 //0.46 // 0.4
    private PIDFController Ypidf = new PIDFController(0.6,0,0,0.5,0.1);
    private PIDFController Rpidf = new PIDFController(0.9,0,0,8,0.2);
    //0.9 : 0.000008 : 0

    private final double PATH_TOLERANCE = 4;
    private final double PATH_ROTATION_TOLERANCE = Math.PI / 2;

    public void runOpMode()
    {
        mapHardware();
        resetDriveEncoder();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Velocity: ", 0);
        telemetry.addData("Target Velocity: ", 0);

        kaiOdo = new _7198_OdoController();
        kaiOdo.initializeHardware(hardwareMap);
        //disLtoR tuning log
        //39.37 -> 40deg for 90deg
        //37.37 -> 30deg for 90deg
        //39.37 -> 87deg for 90deg
        //39.87 -> 87deg for 90deg
        //40.87 -> 85deg for 90deg
        //38.8 -> aprox 90deg
        //40 -> 85deg
        //38.6 -> 89.2deg
        //38.3 -> 89.6deg
        //38 -> 90.3 / 90.8
        //38.15 -> 89.7 / 90.7
        //38.2 -> 90.1 / 90.6 / 91.4 / 91.2
        //38.05 -> 91.2 / 90.9
        //38.35 -?> 89.8 /89.8
        //38.325 -> 90 / 90.6 / 90.2 / 89 / 89.1

        //disMidtoC tuning log
        //29.21 -> 86.7deg of 90deg
        //28.73375 -> 87deg of 90deg
        //28.45 -> 89.2 / 89.2
        //28 -> 89.1 / 89.1
        //29 -> 89.6 / 89.6 / 90 / 90.1

        //ALL Tune tuning log
        //38.325 and 29 -> 90 / 90.6 / 90.2 / 89 / 89.1 / 89.6 / 89.6 / 90 / 90.1 avr(89.8)
        //38.325 and 29.2 -> 89.4 / 88.8
        //38.315 and 29.2 -> 88.6
        //38.325 and 28.9 -> 89.4 / 89.5
        //38.325 and 29.02 -> 89.6 / 88.6
        //38.325 and 28.95 -> 88 / 89.1
        //38.32 and 29 -> 89.6 / 89.6 / 91.2
        //38.315 and 29.1 -> 89.3 / 91.1
        //38.31 and 29.1 -> 89.6 / 90.1
        //close but not there yet.

        //pixelPlacer.setTargetPosition(-50);
        //pixelPlacer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //pixelPlacer.setPower(-0.5);


        visProcessor = new Team7198PropProcessor(false, this);

        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), visProcessor);

        //visionPortal.resumeStreaming();
        visionPortal.resumeLiveView();

        //FtcDashboard.getInstance().startCameraStream(visProcessor, 0);

        Xpidf.reset();
        Ypidf.reset();
        Rpidf.reset();
        Xpidf.launch(0,System.currentTimeMillis());
        Ypidf.launch(0,System.currentTimeMillis());
        Rpidf.launch(0,System.currentTimeMillis());

        int currentPathIndex = 0;

        int fileId = hardwareMap.appContext.getResources().getIdentifier("forward_test_path", "raw", hardwareMap.appContext.getPackageName());
        Odo6Path autonomousPath = new Odo6Path(hardwareMap.appContext.getResources().openRawResource(fileId), telemetry);

        int fileId1 = hardwareMap.appContext.getResources().getIdentifier("path_recorder_output", "raw", hardwareMap.appContext.getPackageName());
        Odo6Path autonomousPathAlt1 = new Odo6Path(hardwareMap.appContext.getResources().openRawResource(fileId), telemetry);
        int fileId2 = hardwareMap.appContext.getResources().getIdentifier("path_recorder_output", "raw", hardwareMap.appContext.getPackageName());
        Odo6Path autonomousPathAlt2 = new Odo6Path(hardwareMap.appContext.getResources().openRawResource(fileId), telemetry);
        int fileId3 = hardwareMap.appContext.getResources().getIdentifier("path_recorder_output", "raw", hardwareMap.appContext.getPackageName());
        Odo6Path autonomousPathAlt3 = new Odo6Path(hardwareMap.appContext.getResources().openRawResource(fileId), telemetry);

        waitForStart();
        while(opModeIsActive())
        {
            kaiOdo.update();

            PoseVelocity2D pathPosition = autonomousPath.getPosition(currentPathIndex);

            if(pathPosition != null)
            {
                //telemetry.addLine(Math.round(pathPosition.getX() *10)/10d + " : " + Math.round(pathPosition.getY() *10)/10d + " : " + Math.round(pathPosition.getR() *10)/10d);

                if(traverseToPosition(pathPosition, new PoseVelocity2D(kaiOdo.getX(), kaiOdo.getY(), kaiOdo.getHeadingRad(), kaiOdo.getDeltaPose().getY(), kaiOdo.getDeltaPose().getX(), kaiOdo.getDeltaPose().getHeadingRad())))
                {
                    currentPathIndex++;
                    if(currentPathIndex < autonomousPath.length() - 1)
                        while(checkIfNear(autonomousPath.getPosition(currentPathIndex), new PoseVelocity2D(kaiOdo.getX(), kaiOdo.getY(), kaiOdo.getHeadingRad(), kaiOdo.getDeltaPose().getY(), kaiOdo.getDeltaPose().getX(), kaiOdo.getDeltaPose().getHeadingRad())))
                        {
                            currentPathIndex++;
                            if(currentPathIndex >= autonomousPath.length())
                                break;
                        }

                    telemetry.addData("Velocity: ", kaiOdo.getDeltaPose().getHeadingRad());
                    telemetry.addData("Target Velocity: ", pathPosition.getVheadingRad());
                }
                //telemetry.addLine(pathPosition.toString());
                telemetry.addLine("Path index: " + currentPathIndex + "/" + autonomousPath.length());
            }
            else
            {
                traverseToPosition(autonomousPath.getPosition(autonomousPath.length() - 1), new PoseVelocity2D(kaiOdo.getX(), kaiOdo.getY(), kaiOdo.getHeadingRad(), kaiOdo.getDeltaPose().getY(), kaiOdo.getDeltaPose().getX(), kaiOdo.getDeltaPose().getHeadingRad()));
                telemetry.addLine("done");
            }

            telemetry.addLine("" + visionData);
            odoTelemetry();
        }

        visionPortal.close();
    }

    public boolean traverseToPosition(PoseVelocity2D target, PoseVelocity2D currentPosition)
    {
        double rotation = Rpidf.update(target.getHeadingRad(),currentPosition.getHeadingRad(), target.getVheadingRad(),System.currentTimeMillis());
        double x = Xpidf.update(target.getX(),currentPosition.getX(), target.getVx(), System.currentTimeMillis());
        double y = Ypidf.update(target.getY(), currentPosition.getY(), target.getVy(), System.currentTimeMillis());
        //double y = 0; double x = 0;
        performGlobalMovement(x,y,rotation);

        telemetry.addLine("IsInBetween Rotation output: " + isInBetween(currentPosition.getHeadingRad(), currentPosition.getHeadingRad() - currentPosition.getVheadingRad(), target.getHeadingRad(), PATH_TOLERANCE));

        return checkIfNear(target, currentPosition);
        //return currentPosition.distance(target) + Math.abs(currentRadRot - target.getR());
    }

//    public boolean checkIfNear(PoseVelocity2D target, PathMarker currentPosition)
//    {
//        if(target != null) {
//            boolean clear = isInBetween(currentPosition.getX(), currentPosition.getX() - currentPosition.getVx(), target.getX(), PATH_TOLERANCE)
//                    && isInBetween(currentPosition.getY(), currentPosition.getY() - currentPosition.getVy(), target.getY(), PATH_TOLERANCE)
//                    && isInBetween(currentPosition.getR(), currentPosition.getR() - currentPosition.getVr(), target.getHeadingRad(), PATH_TOLERANCE)
//                    ;
//            return clear;
//        }
//        return true;
//    }

    public boolean checkIfNear(PoseVelocity2D target, PoseVelocity2D currentPosition)
    {
        if(target != null)
        {
            boolean clear =
                    target.distance(currentPosition) < PATH_TOLERANCE
                    && isInBetween(currentPosition.getHeadingRad(), currentPosition.getHeadingRad() - currentPosition.getVheadingRad(), target.getHeadingRad(), PATH_ROTATION_TOLERANCE)
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
        xl = (x * Math.cos(kaiOdo.getHeadingRad())) + (y * Math.sin(kaiOdo.getHeadingRad()));
        yl = (x * Math.sin(kaiOdo.getHeadingRad())) - (y * Math.cos(kaiOdo.getHeadingRad()));

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

        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);

    }

    public void odoTelemetry()
    {
        telemetry.addLine("Internal Position:");
        telemetry.addData("(X, Y, Degrees)", Math.round(kaiOdo.getX() *10)/10d + " : " + Math.round(kaiOdo.getY() *10)/10d + " : " + Math.round(kaiOdo.getHeadingRad() *10)/10d);
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
        //pixelPlacer = hardwareMap.get(DcMotorEx.class, "pixelArm");

        encoderBack = rightFront;
        encoderLeft = leftBack;
        encoderRight = leftFront;

        //colorMan = hardwareMap.get(ColorRangeSensor.class, "colorSensor");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        //pixelPlacer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //pixelPlacer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
