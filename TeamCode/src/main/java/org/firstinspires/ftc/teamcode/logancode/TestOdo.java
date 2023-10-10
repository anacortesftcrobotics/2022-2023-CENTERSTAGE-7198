package org.firstinspires.ftc.teamcode.logancode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.kaicode.Odo1;

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

        kaiOdo = new Odo1(38.6,28.73375,3.5,8192);
        //disLtoR
        //39.37 -> 40deg for 90deg
        //37.37 -> 30deg for 90deg
        //39.37 -> 87deg for 90deg
        //39.87 -> 87deg for 90deg
        //40.87 -> 85deg for 90deg
        //38.8 -> aprox 90deg
        //40 -> 85deg
        //38.6 -> 89.2deg
        //37.5 ->

        //disMidtoC
        //29.21 -> 86.7deg of 90deg
        //28.73375 -> 87deg of 90deg

        waitForStart();
        while(opModeIsActive())
        {
            kaiOdo.setEncoderPos(-encoderLeft.getCurrentPosition(),
                    encoderRight.getCurrentPosition(),
                    encoderBack.getCurrentPosition());

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
