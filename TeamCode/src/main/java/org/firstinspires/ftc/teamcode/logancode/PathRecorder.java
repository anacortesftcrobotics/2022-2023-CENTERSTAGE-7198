package org.firstinspires.ftc.teamcode.logancode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.kaicode.Odo1;

import java.io.File;

@TeleOp(name = "PathRecorder_Logan", group = "Debug/Tool")
public class PathRecorder extends LinearOpMode {

    //Motors
    private DcMotor leftBack, leftFront, rightBack, rightFront;

    //Odometry
    private DcMotor encoderRight, encoderLeft, encoderBack;
    private Odo1 kaiOdo;
    double xOld = 0;
    double yOld = 0;
    double rOld = 0;

    public Path path = new Path();

    private PathMarker lastPath;
    private long lastTime;
    //private final double MAX_SPEED = 150; // cm per second. theoretical: 165 cm/s

    @Override
    public void runOpMode() throws InterruptedException {

        mapHardware();
        resetDriveEncoder();
        kaiOdo = new Odo1(36.2,25.8,4.8,2000);

        lastPath = new PathMarker(0,0,0,0,0,0);
        lastTime = System.currentTimeMillis();

        waitForStart();

        while(opModeIsActive())
        {
            kaiOdo.setEncoderPos(-encoderLeft.getCurrentPosition(),
                    encoderRight.getCurrentPosition(),
                    encoderBack.getCurrentPosition());

            double deltaTime = System.currentTimeMillis() - lastTime;
            PathMarker p =  new PathMarker(kaiOdo.getX(), kaiOdo.getY(),kaiOdo.getHDeg(), (kaiOdo.getX() - xOld) * deltaTime, (kaiOdo.getY() - yOld) * deltaTime, (kaiOdo.getHDeg() - rOld) * deltaTime);
            if(p.distance(lastPath) > 0.25 + kaiOdo.getDeltaDistance())
            {
                path.addPathMarker(p);
                telemetry.addLine("New Path Generated at: " + p);
                lastPath = new PathMarker(p.getX(),p.getY());
            }
            telemetry.addLine("Path to path dist: " + p.distance(lastPath));
            telemetry.addLine("Delta val: " + (kaiOdo.getDeltaDistance()));

            lastTime = System.currentTimeMillis();

            telemetry.addLine("Internal Position:");
            telemetry.addData("(X, Y, Degrees)", Math.round(kaiOdo.getX() *10)/10d + " : " + Math.round(kaiOdo.getY() *10)/10d + " : " + Math.round(kaiOdo.getHDeg() *10)/10d);
            telemetry.addLine("Path count: " + path.length());

            xOld = kaiOdo.getX();
            yOld = kaiOdo.getY();
            rOld = kaiOdo.getHDeg();

            if(gamepad1.a)
            {
                String filename = "path_recorder_output.txt";
                File file = AppUtil.getInstance().getSettingsFile(filename);
                ReadWriteFile.writeFile(file, path.serialize());
                telemetry.log().add("saved to '%s'", filename);
            }

            PerformLocalMovement(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);


            telemetry.update();
        }
    }

    public void PerformLocalMovement(double x, double y, double rx)
    {
        double leftBackPower;
        double rightBackPower;
        double leftFrontPower;
        double rightFrontPower;

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

        leftBackPower = (leftBackPower);
        rightBackPower = (rightBackPower);
        leftFrontPower = (leftFrontPower);
        rightFrontPower = (rightFrontPower);

        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);

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
}
