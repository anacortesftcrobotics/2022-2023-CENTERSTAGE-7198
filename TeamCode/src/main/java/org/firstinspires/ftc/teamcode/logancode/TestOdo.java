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

    }

    public void mapHardware()
    {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        encoderBack = hardwareMap.get(DcMotor.class, "encoderBack");
        encoderLeft = hardwareMap.get(DcMotor.class, "encoderLeft");
        encoderRight = hardwareMap.get(DcMotor.class, "encoderRight");

        //colorMan = hardwareMap.get(ColorRangeSensor.class, "colorSensor");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
