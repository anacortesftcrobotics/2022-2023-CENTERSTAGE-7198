package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="CenterStage 8934 TeleOp", group="8934 TeleOp")
public class CenterStageTeleOp extends OpMode
{
    private CenterStageRobot theRobot;
    private MecanumDrive fDrive;
    Motor frontLeftX, frontRightX, backLeftX, backRightX;
    GamepadEx fancyGamePad, fancyDrivePad;
    ButtonReader aReader, lbReader, rbReader, xReader, yReader, bReader, dpad_downReader, dpadUpReader, dpad_leftReader;
    TriggerReader ltReader, rtReader;
    RevIMU imu;
    private boolean fDriveMode = false;
    @Override
    public void init()
    {
        theRobot = new CenterStageRobot(null, hardwareMap);
        fancyGamePad = new GamepadEx(gamepad2);

        ltReader = new TriggerReader(fancyGamePad, GamepadKeys.Trigger.LEFT_TRIGGER);
        rtReader = new TriggerReader(fancyGamePad, GamepadKeys.Trigger.RIGHT_TRIGGER);
        lbReader = new ButtonReader(fancyGamePad, GamepadKeys.Button.LEFT_BUMPER);
        rbReader = new ButtonReader(fancyGamePad, GamepadKeys.Button.RIGHT_BUMPER);
        aReader = new ButtonReader(fancyGamePad, GamepadKeys.Button.A);
        bReader = new ButtonReader(fancyGamePad, GamepadKeys.Button.B);
        xReader = new ButtonReader(fancyGamePad, GamepadKeys.Button.X);
        yReader = new ButtonReader(fancyGamePad, GamepadKeys.Button.Y);
        dpad_downReader = new ButtonReader(fancyGamePad, GamepadKeys.Button.DPAD_DOWN);
        dpad_leftReader = new ButtonReader(fancyGamePad, GamepadKeys.Button.DPAD_LEFT);

        // experimental fDrive
        fancyDrivePad = new GamepadEx(gamepad1);
        dpadUpReader = new ButtonReader(fancyDrivePad, GamepadKeys.Button.DPAD_UP);
        frontLeftX = new Motor(hardwareMap, "frontLeft");
        frontRightX = new Motor(hardwareMap, "frontRight");
        backLeftX = new Motor(hardwareMap, "backLeft");
        backRightX = new Motor(hardwareMap, "backRight");
        fDrive = new MecanumDrive(frontLeftX, frontRightX, backLeftX, backRightX);

        //imu = new RevIMU(hardwareMap);
        //imu.init();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        theRobot.launcherServo.setPosition(.73);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() { }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() { }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop()
    {
        // receive gamepad input
        aReader.readValue();
        ltReader.readValue();
        rtReader.readValue();
        lbReader.readValue();
        rbReader.readValue();
        bReader.readValue();
        xReader.readValue();
        yReader.readValue();
        dpad_downReader.readValue();
        dpadUpReader.readValue();
        dpad_leftReader.readValue();

        if (aReader.isDown()) {
            theRobot.openBucket();
        } else if (aReader.wasJustReleased()) {
            theRobot.closeBucket();
        }

        if (xReader.wasJustReleased()) {
            theRobot.intakeDownGrabPixelsComeUp();
        } else if (bReader.wasJustReleased()) {
            theRobot.loadPixelsInBucket();
        } else if (yReader.wasJustReleased()) {
            theRobot.setIntakeToBatteringRam();
        }

        // drive screw out
        if (ltReader.isDown()) {
            theRobot.hookArm.setPower(fancyGamePad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
        } else if (ltReader.wasJustReleased()) {
            theRobot.hookArm.setPower(0);
        }

        if(dpad_leftReader.wasJustReleased()) {
            theRobot.launcherServo.setPosition(-1);
        }


        // drive screw in
        if (rtReader.isDown()) {
            theRobot.hookArm.setPower(- fancyGamePad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
        } else if (rtReader.wasJustReleased()) {
            theRobot.hookArm.setPower(0);
        }

        if (lbReader.wasJustReleased()) { theRobot.hookElbow.setPosition(0.75); } // hanger up
        if (rbReader.wasJustReleased()) { theRobot.hookElbow.setPosition(0); } // hanger down
        if (dpad_downReader.wasJustReleased()) { theRobot.closeBothFingers(); } // reset pixel picker-uppers

        double viperPower = gamepad2.left_stick_y;
        theRobot.safeViperSlide(viperPower);
        /*
       if(theRobot.bucketStop.getState() == false){
           theRobot.viperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       }
        if(theRobot.viperSlide.getCurrentPosition() >= -2000) {
            theRobot.safeViperSlide(viperPower);
        }
        else if(theRobot.viperSlide.getCurrentPosition() <= -1800){
            theRobot.viperSlide.setPower(0);
        }
        */
        telemetry.addData("Encoder pos: ",theRobot.viperSlide.getCurrentPosition());
        telemetry.addData("bucket stop state: ", theRobot.bucketStop.getState());
        telemetry.update();

        // enable/disable experimental fDrive
        if (dpadUpReader.wasJustReleased()) {
            if (fDriveMode) {
                // return to normal mode
                theRobot.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                theRobot.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                fDriveMode = false;
                gamepad1.rumbleBlips(1);
                telemetry.addLine("drive mode: normal");
            } else {
                // enable fDrive
                theRobot.frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                theRobot.backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                fDriveMode = true;
                gamepad1.rumbleBlips(3);
                telemetry.addLine("drive mode: fDrive");
            }
        }

        if (fDriveMode) {
            fDrive.driveFieldCentric(
                    - fancyDrivePad.getLeftX(), - fancyDrivePad.getLeftY(), -fancyDrivePad.getRightX(),
                    imu.getRotation2d().getDegrees(),
                    false);
        } else {
            theRobot.mecanumX(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
    }
    @Override
    public void stop() { }

}
