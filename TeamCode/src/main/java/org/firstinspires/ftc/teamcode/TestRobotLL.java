package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ResourceBundle;

@TeleOp
public class TestRobotLL extends OpMode
{

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    ColorRangeSensor colorMan;

    double speedCoefficient = 1;

    boolean[] savedStates1 = {false,false};

    @Override
    public void init()
    {

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        colorMan = hardwareMap.get(ColorRangeSensor.class, "colorSensor");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop()
    {
        //leftFront.setPower(0.9);
        Controller1();
        telemetry();
    }

    public void Controller1()
    {
        double leftBackPower;
        double rightBackPower;
        double leftFrontPower;
        double rightFrontPower;

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        updateSpeedCoefficient();

        x = deadZone(x);
        y = deadZone(y);
        rx = deadZone(rx);

        //rx += poleCenter();

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        leftBackPower = (y - x + rx) / denominator;
        rightBackPower = (y + x - rx) / denominator;
        leftFrontPower = (y + x + rx) / denominator;
        rightFrontPower = (y - x - rx) / denominator;
/*
        leftBackPower = Math.cbrt(leftBackPower);
        rightBackPower = Math.cbrt(rightBackPower);
        leftFrontPower = Math.cbrt(leftFrontPower);
        rightFrontPower = Math.cbrt(rightFrontPower);

        leftBackPower = (leftBackPower * speedCoefficient * 0.75);
        rightBackPower = (rightBackPower * speedCoefficient * 0.75);
        leftFrontPower = (leftFrontPower  * speedCoefficient * 0.75);
        rightFrontPower = (rightFrontPower * speedCoefficient * 0.75);
*/
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);

    }
    public void telemetry()
    {
        telemetry.addData("Color Data: ", colorMan.red() + " " + colorMan.green() + " " + colorMan.blue());
        telemetry.update();
    }

    public void updateSpeedCoefficient()
    {
        //Speed coefficient dpad up n down
        if(gamepad1.dpad_up)
        {
            if(!savedStates1[0])
            {
                if(speedCoefficient < 1)
                    speedCoefficient += 0.2;
                savedStates1[1] = true;
            }
        }
        else
        {
            savedStates1[1] = false;
        }
        if(gamepad1.dpad_down)
        {
            if(!savedStates1[0])
            {
                if(speedCoefficient > 0.2)
                    speedCoefficient -= 0.2;
                savedStates1[0] = true;
            }
        }
        else
        {
            savedStates1[0] = false;
        }
    }

    public double deadZone(double input)
    {
        if (Math.abs(input) < 0.1)
        {
            return  0;
        }
        else
        {
            return input;
        }
    }
}