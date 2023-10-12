package org.firstinspires.ftc.teamcode.logancode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.kaicode.Odo1;
import org.firstinspires.ftc.teamcode.kaicode.PIDFController;

import java.io.File;

@Autonomous(name = "TestOdo_Logan", group = "Autos")
public class TestOdo extends LinearOpMode {

    //Motors
    private DcMotor leftBack, leftFront, rightBack, rightFront, lift, leadScrew;

    //Odometry
    private DcMotor encoderRight, encoderLeft, encoderBack;
    private Odo1 kaiOdo;

    public void runOpMode()
    {
        mapHardware();
        resetDriveEncoder();

        kaiOdo = new Odo1(38.31,29.1,3.5,8192);
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

        PIDFController Xpidf = new PIDFController(1,0,0,0.66,0.33);
        PIDFController Ypidf = new PIDFController(1,0,0,0.66,0.33);
        PIDFController Rpidf = new PIDFController(1,0,0,0.33,0.11);
        Xpidf.reset();
        Ypidf.reset();
        Rpidf.reset();
        Xpidf.launch(0,System.currentTimeMillis());
        Ypidf.launch(0,System.currentTimeMillis());
        Rpidf.launch(0,System.currentTimeMillis());

        //pidf.launch();

        int currentPathPos = 0;

        int fileId = hardwareMap.appContext.getResources().getIdentifier("test_path", "raw", hardwareMap.appContext.getPackageName());
        Path autonomousPath = new Path(hardwareMap.appContext.getResources().openRawResource(fileId), telemetry);

        waitForStart();
        while(opModeIsActive())
        {
            kaiOdo.setEncoderPos(-encoderLeft.getCurrentPosition(),
                    encoderRight.getCurrentPosition(),
                    encoderBack.getCurrentPosition());

            telemetry.addLine("" + autonomousPath.getPosition(currentPathPos));

            sleep(1000);
            currentPathPos++;

            telemetry();
        }
    }

    public void mapHardware()
    {
        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        leftBack = hardwareMap.get(DcMotor.class, "backLeft");
        rightBack = hardwareMap.get(DcMotor.class, "backRight");

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



    public void telemetry()
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
}
