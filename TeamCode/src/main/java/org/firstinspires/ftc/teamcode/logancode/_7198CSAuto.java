package org.firstinspires.ftc.teamcode.logancode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.logancode._7198CSAuto;
import org.firstinspires.ftc.teamcode.logancode._7198CSRobot;
import org.firstinspires.ftc.teamcode.logancode._7198CSVisionManager;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class _7198CSAuto extends LinearOpMode {
    final double DESIRED_DISTANCE = 8; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN = 0.018;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.03;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.2;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.2;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.2;   //  Clip the turn speed to this max value (adjust for your robot)

    private int AutoState = 1;

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
            theRobot.fingerRight.setPosition(1);
            theRobot.fingerLeft.setPosition(0);
        }

        waitForStart(); // ready to rock

        while (opModeIsActive()) {
            switch (AutoState) {
                case 1:
                    if (this.TeamPropLocation == -1) {
                        this.TeamPropLocation = visionManager.getDetectedSpikeMark();
                        telemetry.addData("auto", "detected prop = %d", this.TeamPropLocation);
                        AutoState++;
                    }
                    break;
                case 2:
                    telemetry.addData("case 2 started", this.TeamPropLocation);
                    telemetry.update();
                    if (scorePurplePixelandTurn(this.TeamPropLocation))
                        AutoState++;
                    telemetry.addData("case 2 finished", this.TeamPropLocation);
                    telemetry.update();
                    break;
                case 3:
                    visionManager.enableAprilTagProcessor();
                    AutoState++;
                    break;
                case 4:
                    theRobot.closeBothFingers();
                    if (!arrivedAtAprilTag) {
                        // this function will occur repeatedly as the robot hones in on it
                        if (THIS_ALLIANCE == _7198CSAuto.ALLIANCE.RED) {
                            telemetry.addLine("going to red april tag");
                            if (this.TeamPropLocation == 3) {
                                driveToAprilTag(4);
                                telemetry.addLine("going to red april tag 4");
                            }
                            else if (this.TeamPropLocation == 1) {
                                driveToAprilTag(5);
                                telemetry.addLine("going to red april tag 5");
                            }
                            else if (this.TeamPropLocation == 2) {
                                driveToAprilTag(6);
                                telemetry.addLine("going to red april tag 6");
                            }
                            /*else {
                                driveToAprilTag(5);
                            }*/
                        }
                        else if (THIS_ALLIANCE == _7198CSAuto.ALLIANCE.BLUE) {
                            if (this.TeamPropLocation == 3) {
                                driveToAprilTag(1);
                            }
                            else if (this.TeamPropLocation == 1) {
                                driveToAprilTag(2);
                            }
                            else if (this.TeamPropLocation == 2) {
                                driveToAprilTag(3);
                            }
                            else {
                                driveToAprilTag(2);
                            }
                        }

                       /* if (THIS_ALLIANCE == _7198CSAuto.ALLIANCE.RED) {
                            driveToAprilTag(this.TeamPropLocation + 4);
                        } else if (THIS_ALLIANCE == _7198CSAuto.ALLIANCE.BLUE) {
                            driveToAprilTag(this.TeamPropLocation + 1);
                        } */
                    } else {
                        scoreYellowPixelandPark(this.TeamPropLocation);


                        requestOpModeStop();
                    }
                    telemetry.update();
                    break;
                default:

                    break;
            }
        }
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
            theRobot.mecanumX(drive/4, strafe/4, 0);
            theRobot.robotArmNap(1,125);
            telemetry.addData("auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }
    }

//    private void driveToAprilTag(int DetectedPropZone)
//    {
//        //double targetAngle = THIS_ALLIANCE == ALLIANCE.BLUE ? 90 : -90;
//
//        if(DetectedPropZone == 1)
//            theRobot.moveRobotPosition_IN(-20,35,0, 125,telemetry);
//        if(DetectedPropZone == 2)
//            theRobot.moveRobotPosition_IN(-29.5,35,0, 125,telemetry);
//        if(DetectedPropZone == 3)
//            theRobot.moveRobotPosition_IN(-37,35,0, 125,telemetry);
//
//        if(DetectedPropZone == 4)
//            theRobot.moveRobotPosition_IN(-37,-35,-0, 125,telemetry);
//        if(DetectedPropZone == 5)
//            theRobot.moveRobotPosition_IN(-29.5,-35,-0, 125,telemetry);
//        if(DetectedPropZone == 6)
//            theRobot.moveRobotPosition_IN(-20,-35,-0, 125,telemetry);
//
//        if(DetectedPropZone < 4)
//            theRobot.setRobotRotation(0,0,90,125,telemetry);
//        else
//            theRobot.setRobotRotation(0,0,-90,125,telemetry);
//
//    }

    private void scoreYellowPixelandPark(int theDangPropZone) {
        // stop moving
        theRobot.halt(125);

        // final approach
        if (THIS_ALLIANCE == ALLIANCE.BLUE) {
            theRobot.drive(0,-0.04,0,200,125);
        }
        else if (THIS_ALLIANCE == ALLIANCE.RED) {
            //theRobot.drive(0,-0.04,0,170,125);
        }
        theRobot.robotArmNap(300,125);
        theRobot.drive(0.05,0,0,600,125);
        // drop the yellow pixel
        theRobot.fingerRight.setPosition(0.6);
        theRobot.robotArmNap(800, 120);
        theRobot.wristServo.setPosition(0.2);
        theRobot.robotArmNap(200, 120);
        theRobot.closeBothFingers();
        theRobot.SetArmAngle(-48);
        theRobot.drive(-0.05, 0, 0, 500,-48);


        // strafe to park
        if (theDangPropZone == 2) {
            theRobot.drive(0, 0.1, 0, 1700,-48);
        } else if (theDangPropZone == 1) {
            theRobot.drive(0, 0.1, 0, 1300,-48);
        } else {
            theRobot.drive(0, 0.1, 0, 1000,-48);
        }
        theRobot.drive(0.05,0,0,500,-48);
        theRobot.halt(-48);
    }

    //returns true when completed the placement
    private boolean scorePurplePixelandTurn(int theDangPropPosition) {
        telemetry.addData("That Dang Prop Position", theDangPropPosition);
        if (theDangPropPosition == 3) {
            scorePurplePixelLeft();
            theRobot.fingerRight.setPosition(1);
            // rotate into AprilTag viewing position
            if (THIS_ALLIANCE == _7198CSAuto.ALLIANCE.RED) {
                theRobot.setRobotRotation(0, 0, -90, 125, telemetry);
            } else if (THIS_ALLIANCE == _7198CSAuto.ALLIANCE.BLUE) {
                theRobot.setRobotRotation(0, 0, 90, 125, telemetry);
            }
        } else if (theDangPropPosition == 1) {
            scorePurplePixelCenter();
            theRobot.fingerRight.setPosition(1);
            // rotate into viewing position
            if (THIS_ALLIANCE == _7198CSAuto.ALLIANCE.RED) {
                theRobot.setRobotRotation(0, 0, -90, 125, telemetry);
            } else if (THIS_ALLIANCE == _7198CSAuto.ALLIANCE.BLUE) {
                theRobot.setRobotRotation(0,0,90, 125, telemetry );
            }
        } else {
            // zone 3
            scorePurplePixelRight();
            theRobot.fingerRight.setPosition(1);
            // back away and rotate into viewing position
            if (THIS_ALLIANCE == _7198CSAuto.ALLIANCE.RED) {
                theRobot.setRobotRotation(0, 0, -90, 125, telemetry);
            } else if (THIS_ALLIANCE == _7198CSAuto.ALLIANCE.BLUE) {
                theRobot.setRobotRotation(0, 0, 90, 125, telemetry);
            }
            //theRobot.moveRobotPosition_IN(-24,-24,0, 125,telemetry);
        }
        //theRobot.setRobotRotation(0,0,0,125,telemetry);
        //theRobot.drive(-1,0,0,100,125);
        theRobot.halt(125);
        return true;
    }


    //returns true when completed
    private void scorePurplePixelCenter() {
        theRobot.SetArmAngle(telemetry,180);
        theRobot.wristServo.setPosition(0);

        theRobot.pixelSlide.setTargetPosition(-2500);
        theRobot.pixelSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        theRobot.pixelSlide.setPower(-1);

        theRobot.robotArmNap(1500,185);
        theRobot.drive(.07,0,0,440, 185);
        theRobot.halt(185);
        theRobot.robotArmNap(1000,185);
        theRobot.fingerLeft.setPosition(0.54);
        theRobot.robotArmNap(300, 185);

        theRobot.pixelSlide.setTargetPosition(0);
        theRobot.pixelSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        theRobot.pixelSlide.setPower(1);
        theRobot.robotArmNap(1500, 125);
        theRobot.drive(0.03,0,0,300, 125);
        theRobot.setIntakeToCameraViewing();
       /* theRobot.drive(0, 0, -.2, 175);
        theRobot.halt();
        theRobot.setIntakeToBatteringRam();


        // overshoot to plow the prop
        theRobot.drive(0.2, 0, 0, 1000);
        // back up to the tape
        theRobot.drive(-0.2, 0, 0, 400);
        theRobot.halt();
        if (!theRobot.placePurplePixel())
            return false;

        // back away
        theRobot.drive(-0.2, 0, 0, 200);
        theRobot.halt();
        theRobot.setIntakeToCameraViewing();
        return true;

        */
    }

    private void scorePurplePixelRight()
    {
        theRobot.SetArmAngle(telemetry,185);
        theRobot.wristServo.setPosition(0);

        theRobot.pixelSlide.setTargetPosition(-2500);
        theRobot.pixelSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        theRobot.pixelSlide.setPower(-1);

        theRobot.robotArmNap(1500,185);
        if (THIS_ALLIANCE == ALLIANCE.RED){
            theRobot.drive(0.03,0,0,300,185);
        }
        else if (THIS_ALLIANCE == ALLIANCE.BLUE){
            theRobot.drive(0.07, 0, 0, 300, 185);
        }
        theRobot.halt(185);
        if (THIS_ALLIANCE == ALLIANCE.RED){
            theRobot.setRobotRotation(0,0,-42,       185,telemetry);
        }
        else if (THIS_ALLIANCE == ALLIANCE.BLUE){
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
        if (THIS_ALLIANCE == ALLIANCE.RED) {
            theRobot.drive(0.03,0,0,200,125);
        }
        else if (THIS_ALLIANCE == ALLIANCE.BLUE) {
            theRobot.drive(0.03, 0, 0, 200, 125);
        }

       /* if (THIS_ALLIANCE == ALLIANCE.RED){
        theRobot.drive(0.04,0,0,650,125);
    }*/
        theRobot.setIntakeToCameraViewing();
    }

    private void scorePurplePixelLeft() {
        // approach
        theRobot.SetArmAngle(telemetry,185);
        theRobot.wristServo.setPosition(0);

        theRobot.pixelSlide.setTargetPosition(-2500);
        theRobot.pixelSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        theRobot.pixelSlide.setPower(-1);

        theRobot.robotArmNap(1500,185);
        theRobot.drive(0.07, 0, 0, 300, 185);
        theRobot.halt(185);
        theRobot.setRobotRotation(0,0,23,       185,telemetry);
        theRobot.fingerLeft.setPosition(0.54);
        theRobot.robotArmNap(300, 185);

        theRobot.pixelSlide.setTargetPosition(0);
        theRobot.pixelSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        theRobot.pixelSlide.setPower(1);
        telemetry.addLine("napping before going to angle 0");
        telemetry.update();
        theRobot.robotArmNap(1500, 125);
        theRobot.setRobotRotation(0,0,0,125, telemetry);
        if (THIS_ALLIANCE == ALLIANCE.RED) {
            theRobot.drive(0.03,0,0,1050,125);
        }
        else if (THIS_ALLIANCE == ALLIANCE.BLUE) {
            theRobot.drive(.03, 0, 0, 230, 125);
        }
        theRobot.setIntakeToCameraViewing();
    }

}
