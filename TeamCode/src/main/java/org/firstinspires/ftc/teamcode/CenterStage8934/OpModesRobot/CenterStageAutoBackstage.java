package org.firstinspires.ftc.teamcode.CenterStage8934.OpModesRobot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CenterStage8934.MiscellaneousStuff.CenterStageVisionManager;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

// this is a generic auto mode for CenterStage Backstage.
// the red and blue alliance game configurations are set by the child classes which inherit from this.
public class CenterStageAutoBackstage extends LinearOpMode
{
    final double DESIRED_DISTANCE = 8; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.01 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.2;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.2;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.2;   //  Clip the turn speed to this max value (adjust for your robot)

    public enum ALLIANCE {
        RED,
        BLUE
    }

    public enum STAGE {
        FRONT,
        BACK
    }

    protected STAGE STAGE_LOCATION;

    protected ALLIANCE THIS_ALLIANCE;

    private int TeamPropLocation = -1; // this means it is not detected
    private AprilTagDetection targetAprilTag; // Used to hold the data for a detected AprilTag
    private CenterStageRobot theRobot;
    private CenterStageVisionManager visionManager;
    private boolean arrivedAtAprilTag = false;
    double lastKnownRangeToAprilTag = 0;

    @Override
    public void runOpMode()
    {
        if (opModeInInit()) {
            initializeGameConfig();
            initializeSystem();
            //theRobot.grabInitialPixels();
        }

        waitForStart(); // ready to rock

        while (opModeIsActive()) {
            if (this.TeamPropLocation == -1) {
                theRobot.grabInitialPixels();
                theRobot.roboNap(200);
                this.TeamPropLocation = visionManager.getDetectedSpikeMark();
                telemetry.addData("auto", "detected prop = %d", this.TeamPropLocation);
                scorePurplePixelandTurn(this.TeamPropLocation);
                visionManager.enableAprilTagProcessor();
            }
                GoThroughTruss();
            if (! arrivedAtAprilTag) {
                // this function will occur repeatedly as the robot hones in on it
                if(THIS_ALLIANCE == ALLIANCE.RED){
                    driveToAprilTag(this.TeamPropLocation + 3);
                }
                else if(THIS_ALLIANCE == ALLIANCE.BLUE){
                    driveToAprilTag(this.TeamPropLocation);
                }
            } else {
                scoreYellowPixelandPark(this.TeamPropLocation);
                if(THIS_ALLIANCE == ALLIANCE.BLUE){
                    theRobot.drive(0,0,.3,1450);
                }
                else if(THIS_ALLIANCE == ALLIANCE.RED){
                    theRobot.drive(0,0,-.3,1250);
                }

                requestOpModeStop();
            }
            telemetry.update();
        }
    }

    private void initializeSystem()
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        visionManager = new CenterStageVisionManager(THIS_ALLIANCE, hardwareMap.get(WebcamName.class, "Webcam 1"));
        theRobot = new CenterStageRobot(this, hardwareMap);
        telemetry.addData("auto", "initializeSystem() complete");
    }

    // empty because the subclass is supposed to implement it
    public void initializeGameConfig() { }

    private void GoThroughTruss()
    {
        if (STAGE_LOCATION == STAGE_LOCATION.FRONT) {
            if (this.TeamPropLocation == 1) {

            }
            else if (this.TeamPropLocation == 2) {

            }
            else if (this.TeamPropLocation == 3) {

            }
        }
    }

    private void driveToAprilTag(int DetectedPropZone)
    {
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
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            // Apply desired axes of motion to the drivetrain.
            theRobot.moveRobot(drive, strafe, turn);
            theRobot.roboNap(20);
            telemetry.addData("auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);}
    }


    private void scoreYellowPixelandPark(int theDangPropZone)
    {
        // stop moving
        theRobot.halt();
        theRobot.setIntakeToAutoScore();
        theRobot.drive(0,.2,0,500);

        // final approach
       theRobot.drive(0.2,0,0,600);

        // drop the yellow pixel
        theRobot.closeRightFinger();
        theRobot.drive(-0.2,0,0,800);


        // strafe to park
        if (theDangPropZone == 1) {
            theRobot.drive(0,-0.4,0,900);
        } else if (theDangPropZone == 2) {
            theRobot.drive(0,-0.4,0,1500);
        } else {
            theRobot.drive(0,-0.4,0,1700);
        }

        // fancy park
        theRobot.drive(0,0,-0.2,300);
        theRobot.drive(0.2,0,0,1000);
        //theRobot.drive(0,0,.3,1500);
        theRobot.halt();
    }

    private void scorePurplePixelandTurn(int theDangPropPosition)
    {
        if (theDangPropPosition == 1) {
            scorePurplePixelLeft();
            // rotate into AprilTag viewing position
            if (STAGE_LOCATION == STAGE_LOCATION.BACK) {
                if (THIS_ALLIANCE == ALLIANCE.RED) {
                    theRobot.drive(0, 0, 0.4, 1500);
                } else if (THIS_ALLIANCE == ALLIANCE.BLUE) {
                    theRobot.drive(0, 0, -0.2, 300);
                }
            }
        }
        else if (theDangPropPosition == 2) {
            scorePurplePixelCenter();
            // rotate into viewing position
            if (STAGE_LOCATION == STAGE_LOCATION.BACK) {
                if (THIS_ALLIANCE == ALLIANCE.RED) {
                    theRobot.drive(0, 0, 0.4, 750);
                } else if (THIS_ALLIANCE == ALLIANCE.BLUE) {
                    theRobot.drive(0, 0, -0.4, 600);
                }
            }
        }
        else {
            // zone 3
            scorePurplePixelRight();
            // back away and rotate into viewing position
            if (STAGE_LOCATION == STAGE_LOCATION.BACK) {
                if (THIS_ALLIANCE == ALLIANCE.RED) {
                    theRobot.drive(-0.2, 0, 0, 600);
                    theRobot.drive(0, 0.3, 0, 1000);
                    theRobot.drive(0, 0, .2, 500);
                } else if (THIS_ALLIANCE == ALLIANCE.BLUE) {
                    theRobot.drive(-0.2, 0, 0, 1000);
                    theRobot.drive(0, -0.3, 0, 500);
                    theRobot.drive(0, 0, -0.4, 1300);
                }
            }
        }

        theRobot.halt();
    }

    private void scorePurplePixelCenter()
    {
        theRobot.drive(0.5,0,0,510);
        theRobot.halt();
        theRobot.drive(0,0,-.2,175);
        theRobot.halt();
        theRobot.setIntakeToBatteringRam();


        // overshoot to plow the prop
        theRobot.drive(0.2,0,0,1000);
        // back up to the tape
        theRobot.drive(-0.2,0,0,400);
        theRobot.halt();
        theRobot.placePurplePixel();

        // back away
        theRobot.drive(-0.2,0,0,200);
        theRobot.setIntakeToCameraViewing();
    }
    private void scorePurplePixelRight()
    {
        theRobot.drive(0.5,0,0,650);
        theRobot.halt();
        theRobot.setIntakeToBatteringRam();

        // turn to the spike mark
        theRobot.drive(0,0,0.3,1000);
        // plow and retreat
        theRobot.drive(0.2,0,0,675);
        theRobot.drive(-0.2,0,0,600);
        // fine positioning
        theRobot.drive(0,0,-0.2,400);
        theRobot.drive(.2,0,0,100);
        theRobot.halt();

        theRobot.placePurplePixel();
        theRobot.setIntakeToCameraViewing();
    }
    private void scorePurplePixelLeft()
    {
        // approach
        theRobot.drive(.5,0, 0, 500);
        theRobot.halt();

        theRobot.setIntakeToBatteringRam();

        // final position on the spike mark
        theRobot.drive(0,0,-0.3,1000);

        // plow and retreat
        theRobot.drive(0.2,0,0,657);
        theRobot.drive(-0.2,0,0,300);
        // fine positioning
        theRobot.drive(0,0,-0.2,400);
        theRobot.halt();

        theRobot.placePurplePixel();
        theRobot.setIntakeToCameraViewing();

        // back away
        theRobot.drive(-0.2,0,0,500);
        theRobot.drive(0,-0.3,0,800);
    }

}
