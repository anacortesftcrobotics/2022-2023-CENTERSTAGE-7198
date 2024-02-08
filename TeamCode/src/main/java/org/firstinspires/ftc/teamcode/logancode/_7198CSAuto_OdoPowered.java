package org.firstinspires.ftc.teamcode.logancode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class _7198CSAuto_OdoPowered extends LinearOpMode {
    final double DESIRED_DISTANCE = 8; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.012;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.2;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.2;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.2;   //  Clip the turn speed to this max value (adjust for your robot)

    public enum ALLIANCE {
        RED,
        BLUE
    }

    protected _7198CSAuto.ALLIANCE THIS_ALLIANCE;

    private int TeamPropLocation = -1; // this means it is not detected
    private AprilTagDetection targetAprilTag; // Used to hold the data for a detected AprilTag
    private _7198CSRobot theRobot;
    private _7198CSVisionManager visionManager;
    private boolean arrivedAtAprilTag = false;
    double lastKnownRangeToAprilTag = 0;

    @Override
    public void runOpMode() {
        if (opModeInInit()) {
            initializeGameConfig();
            initializeSystem();
            theRobot.grabInitialPixels(-48);
        }

        waitForStart(); // ready to rock

        if (this.TeamPropLocation == -1)
        {
            this.TeamPropLocation = visionManager.getDetectedSpikeMark();
            telemetry.addData("auto", "detected prop = %d", this.TeamPropLocation);
        }

        scorePurplePixel(this.TeamPropLocation);

        theRobot.moveRobotPosition_IN(-2,0,-90,-48);
        theRobot.moveRobotPosition_IN(-2,77,-90,-48);
        theRobot.moveRobotPosition_IN(-28,77,-90,-48);
        theRobot.moveRobotPosition_IN(-2,77,-90,-48);
        theRobot.moveRobotPosition_IN(-2,0,-90,-48);
        theRobot.setRobotRotation(0,0,0,-48,telemetry);



    }

    private void initializeSystem() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        visionManager = new _7198CSVisionManager(THIS_ALLIANCE, hardwareMap.get(WebcamName.class, "Webcam 1"));
        theRobot = new _7198CSRobot(hardwareMap, this);
        telemetry.addData("auto", "initializeSystem() complete");
    }

    // empty because the subclass is supposed to implement it
    public void initializeGameConfig() {
    }

    private void driveToAprilTag(int DetectedPropZone) {
        double drive = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)

        targetAprilTag = visionManager.getDetectedAprilTag(DetectedPropZone);

        if (targetAprilTag != null) {
            // "error" here refers to the remaining distance to target.
            double rangeError = (targetAprilTag.ftcPose.range - DESIRED_DISTANCE);
            double headingError = targetAprilTag.ftcPose.bearing;
            double yawError = targetAprilTag.ftcPose.yaw;

            if (rangeError <= DESIRED_DISTANCE && rangeError <= lastKnownRangeToAprilTag) {
                arrivedAtAprilTag = true;
            } else if (rangeError > lastKnownRangeToAprilTag) {
                lastKnownRangeToAprilTag = rangeError;
            }

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            strafe = Range.clip(headingError * STRAFE_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            turn = Range.clip(yawError * TURN_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            // Apply desired axes of motion to the drivetrain.
            theRobot.mecanumX(drive/4, strafe/4, turn/4);
            theRobot.kaiOdo.update();
            theRobot.robotArmNap(10,125);
            telemetry.addData("auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }
    }

    private boolean scorePurplePixel(int theDangPropPosition)
    {
        telemetry.addData("That Dang Prop Position", theDangPropPosition);
        telemetry.update();
        if (theDangPropPosition == 3)
        {
            telemetry.addLine("Left");
            telemetry.update();
            scorePurplePixelLeft();
        }
        else if (theDangPropPosition == 1)
        {
            telemetry.addLine("Center");
            telemetry.update();
            //scorePurplePixelCenter();
        }
        else
        {
            telemetry.addLine("Right");
            telemetry.update();
            //scorePurplePixelRight();
        }

        theRobot.setRobotRotation(0, 0, THIS_ALLIANCE == _7198CSAuto.ALLIANCE.RED ? -90 : 90, 125, telemetry);
        theRobot.halt(125);
        return true;
    }

    private void scoreYellowPixelandPark(int theDangPropZone) {
        // stop moving
        theRobot.halt(125);

        // final approach
        if (THIS_ALLIANCE == _7198CSAuto.ALLIANCE.BLUE) {
            theRobot.moveRobotPosition_IN(0,-0.04,0,125);
        }
        else if (THIS_ALLIANCE == _7198CSAuto.ALLIANCE.RED) {
            //theRobot.moveRobotPosition_IN(0,-0.04,0,170,125);
        }
        theRobot.robotArmNap(300,125);
        theRobot.drive(0.05,0,0, 0,125);
        // drop the yellow pixel
        theRobot.fingerRight.setPosition(0.6);
        theRobot.robotArmNap(800, 120);
        theRobot.wristServo.setPosition(0.2);
        theRobot.robotArmNap(200, 120);
        theRobot.closeBothFingers();
        theRobot.SetArmAngle(-48);
        theRobot.moveRobotPosition_IN(-0.05, 0, 0, -48);


        // strafe to park
        if (theDangPropZone == 3) {
            theRobot.moveRobotPosition_IN(0, -0.1, 0, -48);
        } else if (theDangPropZone == 1) {
            theRobot.moveRobotPosition_IN(0, -0.1, 0, -48);
        } else {
            theRobot.moveRobotPosition_IN(0, -0.1, 0, -48);
        }
        theRobot.moveRobotPosition_IN(0.05,0,0,-48);
        theRobot.halt(-48);
    }

    //returns true when completed
    private void scorePurplePixelCenter() {
        theRobot.SetArmAngle(telemetry,180);
        theRobot.wristServo.setPosition(0);

        theRobot.pixelSlide.setTargetPosition(-2500);
        theRobot.pixelSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        theRobot.pixelSlide.setPower(-1);

        theRobot.robotArmNap(1500,185);
        theRobot.moveRobotPosition_IN(-16.5,0,0,185);
        theRobot.halt(185);
        theRobot.robotArmNap(1000,185);
        theRobot.fingerLeft.setPosition(0.54);
        theRobot.robotArmNap(300, 185);

        theRobot.pixelSlide.setTargetPosition(0);
        theRobot.pixelSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        theRobot.pixelSlide.setPower(1);

        theRobot.robotArmNap(1500, 125);
        theRobot.moveRobotPosition_IN(6.5,0, 0, 125);
    }

    private void scorePurplePixelRight()
    {
        theRobot.SetArmAngle(telemetry,185);
        theRobot.wristServo.setPosition(0);

        theRobot.pixelSlide.setTargetPosition(-2500);
        theRobot.pixelSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        theRobot.pixelSlide.setPower(-1);

        theRobot.robotArmNap(1500,185);
        if (THIS_ALLIANCE == _7198CSAuto.ALLIANCE.RED){
            theRobot.moveRobotPosition_IN(0.03,0,0,185);
        }
        else if (THIS_ALLIANCE == _7198CSAuto.ALLIANCE.BLUE){
            theRobot.moveRobotPosition_IN(0.07, 0, 0, 185);
        }
        theRobot.halt(185);
        if (THIS_ALLIANCE == _7198CSAuto.ALLIANCE.RED){
            theRobot.setRobotRotation(0,0,-42,       185,telemetry);
        }
        else if (THIS_ALLIANCE == _7198CSAuto.ALLIANCE.BLUE){
            theRobot.setRobotRotation(0,0,-42,       185,telemetry);
        }
        theRobot.fingerLeft.setPosition(0.54);
        theRobot.robotArmNap(300, 185);

        theRobot.pixelSlide.setTargetPosition(0);
        theRobot.pixelSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        theRobot.pixelSlide.setPower(1);
        telemetry.addLine("napping before going to angle 0");
        telemetry.update();
        theRobot.robotArmNap(1500, 125);
        theRobot.setRobotRotation(0,0,0,125,telemetry);
        if (THIS_ALLIANCE == _7198CSAuto.ALLIANCE.RED) {
            theRobot.moveRobotPosition_IN(0.04,0,0,125);
        }
        else if (THIS_ALLIANCE == _7198CSAuto.ALLIANCE.BLUE) {
            theRobot.moveRobotPosition_IN(0.04, 0, 0, 125);
        }

       /* if (THIS_ALLIANCE == ALLIANCE.RED){
        theRobot.moveRobotPosition_IN(0.04,0,0,650,125);
    }*/
        theRobot.setIntakeToCameraViewing();
    }

    private void scorePurplePixelLeft() {
        // approach
        theRobot.moveRobotPosition(-12.5,0,0,185);
        theRobot.SetArmAngle(telemetry,185);
        theRobot.wristServo.setPosition(0);

        theRobot.pixelSlide.setTargetPosition(-2500);
        theRobot.pixelSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        theRobot.pixelSlide.setPower(-1);
        theRobot.robotArmNap(1500,185);

        theRobot.setRobotRotation(0,0,23,       185,telemetry);
        theRobot.fingerLeft.setPosition(0.54);
        theRobot.robotArmNap(300, 185);
        theRobot.fingerRight.setPosition(1);

        theRobot.pixelSlide.setTargetPosition(0);
        theRobot.pixelSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        theRobot.pixelSlide.setPower(1);
        theRobot.robotArmNap(1500, -48);

        theRobot.setRobotRotation(0,0,0,125, telemetry);

    }

}
