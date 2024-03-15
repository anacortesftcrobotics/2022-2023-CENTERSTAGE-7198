package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RRdrive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RRtrajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.teamcode.CenterStageFieldConstants8934;

import java.util.Vector;

import static org.firstinspires.ftc.teamcode.CenterStageFieldConstants8934.*;

// this is a generic auto mode for CenterStage Backstage.
// the red and blue alliance game configurations are set by the child classes which inherit from this.
public class CenterStageAutoBackstageOdoPowered extends LinearOpMode {
    final double DESIRED_DISTANCE = 8; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN = 0.75/20;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.5/30;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.5/35;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.2;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.2;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.2;   //  Clip the turn speed to this max value (adjust for your robot)

    public enum STAGE {
        FRONT,
        BACK
    }

    protected STAGE STAGE_LOCATION;

    protected CenterStageAutoBackstage.ALLIANCE THIS_ALLIANCE;

    private int TeamPropLocation = -1; // this means it is not detected
    private AprilTagDetection targetAprilTag; // Used to hold the data for a detected AprilTag
    private CenterStageRobot theRobot;
    private CenterStageVisionManager visionManager;
    private boolean arrivedAtAprilTag = false;
    double lastKnownRangeToAprilTag = 0;
    private OdoControllerAlfalfa kaiOdo;
    private static Vector2d addTest = new Vector2d(4,1);
    public Pose2d STACK3_POSITION = new Pose2d(pixelStack3.plus(addTest), Math.toRadians(180));
    public Vector2d STACK1_POSITION_VECTOR2D = (pixelStack1.plus(addTest));
    private static Vector2d boardAddCenter = new Vector2d(-5.5,0);
    private static Vector2d boardAddLeft = new Vector2d(-6,6.5);
    private static Vector2d blueBoardScoringLeft = new Vector2d(54.5,43.5);
    private static Vector2d blueBoardScoringMiddle = new Vector2d(54.5,41);
    private static Vector2d blueBoardScoringRight = new Vector2d(55,30.75);
    private static Pose2d blueBoardScoring = new Pose2d(blueBoard.plus(boardAddCenter), Math.toRadians(180));
    private static Pose2d centerField = new Pose2d(0, 0, Math.toRadians(0));
    private static Vector2d centerFieldVector = new Vector2d(0,6);
    private static Pose2d centerFieldOffset = new Pose2d(-36,-2,Math.toRadians(180));
    private static Pose2d centerFieldOffset2 = new Pose2d(-4,-2,Math.toRadians(180));


    @Override
    public void runOpMode() {
        if (opModeInInit()) {
            initializeGameConfig();
            initializeSystem();
            theRobot.grabInitialPixelsOdo();
        }

        waitForStart(); // ready to rock

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        //LEFT TAPE MARKER**********

        Trajectory Trajectory1L = drive.trajectoryBuilder(backstageBlueStarting)
                //.splineTo(new Vector2d(14.5, 32), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(29,33.5, Math.toRadians(180)))
                .addTemporalMarker(0.2, () -> {
                    // This marker runs two seconds into the trajectory
                    theRobot.setIntakeToBatteringRam();
                    // Run your action in here!
                })
                .build();

        Trajectory goRightL = drive.trajectoryBuilder(Trajectory1L.end())
                .lineToConstantHeading(new Vector2d(34,43))
                .build();


        Trajectory goToBoardLeft = drive.trajectoryBuilder(goRightL.end())
                .lineToConstantHeading(blueBoardScoringLeft.plus(new Vector2d(0,-3)))
                .build();


        Trajectory goToZeroL = drive.trajectoryBuilder(goToBoardLeft.end())
                .lineToConstantHeading(centerFieldVector)
                .build();

        //LEFT POSITION FRONTSTAGE**********

        Trajectory Trajectory1LF = drive.trajectoryBuilder(frontstageBlueStarting)
                //.splineTo(new Vector2d(14.5, 32), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(-36,33.5, Math.toRadians(180)))
                .addTemporalMarker(0.2, () -> {
                    // This marker runs two seconds into the trajectory
                    theRobot.setIntakeToBatteringRam();
                    // Run your action in here!
                })
                .build();

        Trajectory backUpToLineLF = drive.trajectoryBuilder(Trajectory1LF.end())
                .lineToConstantHeading(new Vector2d(-17, 33.5))
                .build();

        Trajectory backUpToBoardLF = drive.trajectoryBuilder(backUpToLineLF.end())
                .lineToConstantHeading(new Vector2d(34,33.5))
                .build();

        Trajectory goRightLF = drive.trajectoryBuilder(backUpToBoardLF.end())
                .lineToConstantHeading(new Vector2d(34,43))
                .build();


        Trajectory goToBoardLeftF = drive.trajectoryBuilder(goRightLF.end())
                .lineToConstantHeading(blueBoardScoringLeft)
                .build();


        Trajectory goToZeroLF = drive.trajectoryBuilder(goToBoardLeftF.end())
                .lineToConstantHeading(centerFieldVector)
                .build();

        //MIDDLE TAPE MARKER**********

        Trajectory Trajectory1M = drive.trajectoryBuilder(backstageBlueStarting)
                //.splineTo(new Vector2d(14.5, 32), Math.toRadians(0))
                .lineTo(new Vector2d(12,31.75))
                .addTemporalMarker(0.2, () -> {
                    // This marker runs two seconds into the trajectory
                    theRobot.setIntakeToBatteringRam();
                    // Run your action in here!
                })
                .build();

        Trajectory goRightM = drive.trajectoryBuilder(Trajectory1M.end())
                .lineToLinearHeading(new Pose2d(20,32,Math.toRadians(180)))
                .build();


        Trajectory goToBoardMiddle = drive.trajectoryBuilder(goRightM.end())
                .lineToConstantHeading(blueBoardScoringMiddle)
                .build();


        Trajectory goToZeroM = drive.trajectoryBuilder(goToBoardMiddle.end())
                .lineToConstantHeading(centerFieldVector)
                .build();

        //MIDDLE POSITION FRONTSTAGE**********

        Trajectory Trajectory1MF = drive.trajectoryBuilder(frontstageBlueStarting)
                //.splineTo(new Vector2d(14.5, 32), Math.toRadians(0))
                .lineTo(new Vector2d(-36,31.75))
                .addTemporalMarker(0.2, () -> {
                    // This marker runs two seconds into the trajectory
                    theRobot.setIntakeToBatteringRam();
                    // Run your action in here!
                })
                .build();

        Trajectory backUpToLineMF = drive.trajectoryBuilder(Trajectory1MF.end())
                .lineToLinearHeading(new Pose2d(-30, 34.5, Math.toRadians(180)))
                .build();

        Trajectory backUpToBoardMF = drive.trajectoryBuilder(backUpToLineMF.end())
                .lineToConstantHeading(new Vector2d(34,34.5))
                .build();


        Trajectory goRightMF = drive.trajectoryBuilder(backUpToBoardMF.end())
                .lineToLinearHeading(new Pose2d(20,32,Math.toRadians(180)))
                .build();


        Trajectory goToBoardMiddleF = drive.trajectoryBuilder(goRightM.end())
                .lineToConstantHeading(blueBoardScoringMiddle)
                .build();


        Trajectory goToZeroMF = drive.trajectoryBuilder(goToBoardMiddle.end())
                .lineToConstantHeading(centerFieldVector)
                .build();

        //RIGHT TAPE MARKER**********

        Trajectory Trajectory1R = drive.trajectoryBuilder(backstageBlueStarting)
                //.splineTo(new Vector2d(14.5, 32), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(7,33.5, Math.toRadians(180)))
                .addTemporalMarker(0.2, () -> {
                    // This marker runs two seconds into the trajectory
                    theRobot.setIntakeToBatteringRam();
                    // Run your action in here!
                })
                .build();

        Trajectory goRightR = drive.trajectoryBuilder(Trajectory1R.end())
                .lineToConstantHeading(new Vector2d(34,27))
                .build();


        Trajectory goToBoardRight = drive.trajectoryBuilder(goRightR.end())
                .lineToConstantHeading(blueBoardScoringRight)
                .build();


        Trajectory goToZeroR = drive.trajectoryBuilder(goToBoardRight.end())
                .lineToConstantHeading(centerFieldVector)
                .build();

        //RIGHT POSITION FRONTSTAGE**********

        Trajectory Trajectory1RF = drive.trajectoryBuilder(frontstageBlueStarting)
                //.splineTo(new Vector2d(14.5, 32), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(-40,33.5, Math.toRadians(180)))
                .addTemporalMarker(0.2, () -> {
                    // This marker runs two seconds into the trajectory
                    theRobot.setIntakeToBatteringRam();
                    // Run your action in here!
                })
                .build();

        Trajectory backUpToLineRF = drive.trajectoryBuilder(Trajectory1RF.end())
                .lineToLinearHeading(new Pose2d(-30, 33.5, Math.toRadians(180)))
                .build();

        Trajectory backUpToBoardRF = drive.trajectoryBuilder(backUpToLineRF.end())
                .lineToConstantHeading(new Vector2d(34,33.5))
                .build();


        Trajectory goRightRF = drive.trajectoryBuilder(backUpToBoardRF.end())
                .lineToConstantHeading(new Vector2d(34,27))
                .build();


        Trajectory goToBoardRightF = drive.trajectoryBuilder(goRightRF.end())
                .lineToConstantHeading(blueBoardScoringRight.plus(new Vector2d(0,-2)))
                .build();


        Trajectory goToZeroRF = drive.trajectoryBuilder(goToBoardRightF.end())
                .lineToConstantHeading(centerFieldVector)
                .build();

        //GENERAL PATHS**********

        Trajectory goToStackTurn = drive.trajectoryBuilder(new Pose2d(centerFieldVector, Math.toRadians(180)))
                .lineToLinearHeading(STACK3_POSITION)
                .build();


        Trajectory goUnderTrussFromStack = drive.trajectoryBuilder(goToStackTurn.end())
                .lineToLinearHeading(centerField.plus(centerFieldOffset2))
                .build();


        Trajectory goToBoardFromCenter = drive.trajectoryBuilder(centerField.plus(centerFieldOffset2))
                .lineToLinearHeading(blueBoardScoring.plus(new Pose2d(-3,0,Math.toRadians(0))))
                .build();

        Trajectory approachBoard = drive.trajectoryBuilder(goToBoardFromCenter.end())
                .back(4)
                .build();






        if (isStopRequested()) return;


        if (this.TeamPropLocation == -1)
        {
            telemetry.addLine("Seeking prop, please wait...");
            telemetry.update();
        }
        this.TeamPropLocation = visionManager.getDetectedSpikeMark();
        telemetry.clearAll();
        telemetry.addData( "detected prop = %d", this.TeamPropLocation);
        telemetry.update();

        if (STAGE_LOCATION == STAGE.BACK)
        {
            drive.setPoseEstimate(backstageBlueStarting);
            if (this.TeamPropLocation == 1)
            {
                drive.followTrajectory(Trajectory1L);
                theRobot.placePurplePixel();
                drive.followTrajectory(goRightL);
                drive.followTrajectory(goToBoardLeft);
                theRobot.placeAccurateBoard();
                drive.followTrajectory(goToZeroL);
            }

            else if (this.TeamPropLocation == 2)
            {
                drive.followTrajectory(Trajectory1M);
                theRobot.placePurplePixel();
                drive.followTrajectory(goRightM);
                drive.followTrajectory(goToBoardMiddle);
                theRobot.placeAccurateBoard();
                drive.followTrajectory(goToZeroM);
            }

            else if (this.TeamPropLocation == 3)
            {
                drive.followTrajectory(Trajectory1R);
                theRobot.placePurplePixel();
                drive.followTrajectory(goRightR);
                drive.followTrajectory(goToBoardRight);
                theRobot.placeAccurateBoard();
                drive.followTrajectory(goToZeroR);
            }

            else
            {
                drive.followTrajectory(Trajectory1R);
                theRobot.placePurplePixel();
                drive.followTrajectory(goRightR);
                drive.followTrajectory(goToBoardRight);
                theRobot.placeAccurateBoard();
                drive.followTrajectory(goToZeroR);
            }
        }

        else if (STAGE_LOCATION == STAGE.FRONT)
        {
            drive.setPoseEstimate(frontstageBlueStarting);
            if (this.TeamPropLocation == 1)
            {
                drive.followTrajectory(Trajectory1LF);
                drive.followTrajectory(backUpToLineLF);
                theRobot.placePurplePixel();
                drive.followTrajectory(backUpToBoardLF);
                drive.followTrajectory(goRightL);
                drive.followTrajectory(goToBoardLeft);
                theRobot.placeAccurateBoard();
                drive.followTrajectory(goToZeroL);
            }

            else if (this.TeamPropLocation == 2)
            {
                drive.followTrajectory(Trajectory1MF);
                theRobot.placePurplePixel();
                drive.followTrajectory(backUpToLineMF);
                drive.followTrajectory(backUpToBoardMF);
                //drive.followTrajectory(goRightMF);
                drive.followTrajectory(goToBoardMiddleF);
                theRobot.placeAccurateBoard();
                drive.followTrajectory(goToZeroMF);
            }

            else if (this.TeamPropLocation == 3)
            {
                drive.followTrajectory(Trajectory1RF);
                theRobot.placePurplePixel();
                //drive.followTrajectory(backUpToLineRF);
                drive.followTrajectory(backUpToBoardRF);
                drive.followTrajectory(goRightRF);
                drive.followTrajectory(goToBoardRightF);
                theRobot.placeAccurateBoard();
                drive.followTrajectory(goToZeroRF);
            }

            else
            {
                drive.followTrajectory(Trajectory1RF);
                theRobot.placePurplePixel();
                drive.followTrajectory(backUpToLineRF);
                drive.followTrajectory(backUpToBoardRF);
                drive.followTrajectory(goRightRF);
                drive.followTrajectory(goToBoardRightF);
                theRobot.placeAccurateBoard();
                drive.followTrajectory(goToZeroRF);
            }
        }

        else
        {
            telemetry.addLine("No Stage Location.. Uh Oh");
            telemetry.update();
        }



        theRobot.viperRetract();
        drive.followTrajectory(goToStackTurn);
        if (theRobot.CURRENT_COLOR == CenterStageRobot.COLOR.BLUE)
        {
            telemetry.clearAll();
            telemetry.addLine("BLUE DETECTED :)");
            telemetry.update();
            theRobot.roboNap(5000);
            //drive.followTrajectory(goUnderTrussFromStack);
        }
        else
        {
            telemetry.clearAll();
            telemetry.addLine("NO BLUE DETECTION... Uh Oh");
            telemetry.update();
        }
        theRobot.roboNap(1000);
        drive.followTrajectory(goUnderTrussFromStack);
        drive.followTrajectory(goToBoardFromCenter);

        /*
        if (!arrivedAtAprilTag)
        {
            if (THIS_ALLIANCE == CenterStageAutoBackstage.ALLIANCE.RED) {
                driveToAprilTag(5);
            } else if (THIS_ALLIANCE == CenterStageAutoBackstage.ALLIANCE.BLUE) {
                driveToAprilTag(2);
            }
        }
        */

        drive.followTrajectory(approachBoard);
        theRobot.placeBoard();




        while (opModeIsActive()) {


            if (this.TeamPropLocation == -1) {
                theRobot.grabInitialPixels();
                theRobot.roboNap(200);
                this.TeamPropLocation = visionManager.getDetectedSpikeMark();
                telemetry.addData("auto", "detected prop = %d", this.TeamPropLocation);
                //scorePurplePixelandTurn(this.TeamPropLocation);
                visionManager.enableAprilTagProcessor();
                telemetry.addLine("done with pixel scoring");
            }
            GoThroughTruss();
            telemetry.addLine("going to apriltag");
            if (!arrivedAtAprilTag) {
                // this function will occur repeatedly as the robot hones in on it
                if (THIS_ALLIANCE == CenterStageAutoBackstage.ALLIANCE.RED) {
                    driveToAprilTag(this.TeamPropLocation + 3);
                } else if (THIS_ALLIANCE == CenterStageAutoBackstage.ALLIANCE.BLUE) {
                    driveToAprilTag(this.TeamPropLocation);
                }
            }

            requestOpModeStop();
            telemetry.update();
        }
    }

    private void initializeSystem() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        visionManager = new CenterStageVisionManager(THIS_ALLIANCE, hardwareMap.get(WebcamName.class, "Webcam 1"));
        theRobot = new CenterStageRobot(this, hardwareMap);
        telemetry.addData("auto", "initializeSystem() complete");
    }

    // empty because the subclass is supposed to implement it
    public void initializeGameConfig() {
    }

    private void GoThroughTruss() {
        if (STAGE_LOCATION == STAGE_LOCATION.FRONT) {
            if (this.TeamPropLocation == 1) {

            } else if (this.TeamPropLocation == 2) {

            } else if (this.TeamPropLocation == 3) {

            }
        }
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
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            // Apply desired axes of motion to the drivetrain.
            theRobot.moveRobot(drive, strafe, turn);
            theRobot.roboNap(20);
            telemetry.addData("auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }
    }


    private void scoreYellowPixelandPark(int theDangPropZone) {
        // stop moving
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        theRobot.halt();
        theRobot.setIntakeToAutoScore();

        Trajectory strafeyellow = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(2)
                .build();
        Trajectory forwardyellow = drive.trajectoryBuilder(strafeyellow.end())
                .forward(5.5)
                        .build();
        Trajectory backyellow = drive.trajectoryBuilder(forwardyellow.end())
                .back(6)
                .build();

        Trajectory ID1strafe = drive.trajectoryBuilder(backyellow.end())
                .strafeRight(32)
                .build();

        Trajectory ID2strafe = drive.trajectoryBuilder(backyellow.end())
                .strafeLeft(25)
                .build();

        Trajectory ID3strafe = drive.trajectoryBuilder(backyellow.end())
                .strafeLeft(18)
                .build();


        // final approach
       drive.followTrajectory(strafeyellow);
       drive.followTrajectory(forwardyellow);

        // drop the yellow pixel
        theRobot.closeRightFinger();

        drive.followTrajectory(backyellow);
        // strafe to park
        if (theDangPropZone == 1) {
            drive.followTrajectory(ID1strafe);
        } else if (theDangPropZone == 2) {
            drive.followTrajectory(ID2strafe);
        } else {
            drive.followTrajectory(ID3strafe);
        }
    }

    private void scorePurplePixelandTurn(int theDangPropPosition) {
        if (theDangPropPosition == 1) {
            scorePurplePixelLeft();
            // rotate into AprilTag viewing position
            if (STAGE_LOCATION == STAGE_LOCATION.BACK) {
                if (THIS_ALLIANCE == CenterStageAutoBackstage.ALLIANCE.RED) {
                    theRobot.drive(0, 0, 0.4, 1500);
                } else if (THIS_ALLIANCE == CenterStageAutoBackstage.ALLIANCE.BLUE) {
                    theRobot.drive(0, 0, -0.2, 300);
                }
            }
        } else if (theDangPropPosition == 2) {
            scorePurplePixelCenter();
            // rotate into viewing position
            if (STAGE_LOCATION == STAGE_LOCATION.BACK) {
                if (THIS_ALLIANCE == CenterStageAutoBackstage.ALLIANCE.RED) {
                    theRobot.drive(0, 0, 0.4, 750);
                } else if (THIS_ALLIANCE == CenterStageAutoBackstage.ALLIANCE.BLUE) {
                    theRobot.drive(0, 0, -0.4, 600);
                }
            }
        } else {
            // zone 3
            scorePurplePixelRight();
            // back away and rotate into viewing position
            if (STAGE_LOCATION == STAGE_LOCATION.BACK) {
                if (THIS_ALLIANCE == CenterStageAutoBackstage.ALLIANCE.RED) {
                    theRobot.drive(-0.2, 0, 0, 600);
                    theRobot.drive(0, 0.3, 0, 1000);
                    theRobot.drive(0, 0, .2, 500);
                } else if (THIS_ALLIANCE == CenterStageAutoBackstage.ALLIANCE.BLUE) {
                    theRobot.drive(-0.2, 0, 0, 1000);
                    theRobot.drive(0, -0.3, 0, 500);
                    theRobot.drive(0, 0, -0.4, 1300);
                }
            }
        }

        theRobot.halt();
    }

    private void scorePurplePixelCenter() {
        theRobot.drive(0.5, 0, 0, 510);
        theRobot.halt();
        theRobot.drive(0, 0, -.2, 175);
        theRobot.halt();
        theRobot.setIntakeToBatteringRam();


        // overshoot to plow the prop
        theRobot.drive(0.2, 0, 0, 1000);
        // back up to the tape
        theRobot.drive(-0.2, 0, 0, 400);
        theRobot.halt();
        theRobot.placePurplePixel();

        // back away
        theRobot.drive(-0.2, 0, 0, 200);
        theRobot.setIntakeToCameraViewing();
    }

    private void scorePurplePixelRight() {
        theRobot.drive(0.5, 0, 0, 650);
        theRobot.halt();
        theRobot.setIntakeToBatteringRam();

        // turn to the spike mark
        theRobot.drive(0, 0, 0.3, 1000);
        // plow and retreat
        theRobot.drive(0.2, 0, 0, 675);
        theRobot.drive(-0.2, 0, 0, 600);
        // fine positioning
        theRobot.drive(0, 0, -0.2, 400);
        theRobot.drive(.2, 0, 0, 100);
        theRobot.halt();

        theRobot.placePurplePixel();
        theRobot.setIntakeToCameraViewing();
    }

    private void scorePurplePixelLeft() {
        // approach
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory Trajectory1 = drive.trajectoryBuilder(new Pose2d(-60.5,12,Math.toRadians(270)))
                .splineTo(new Vector2d(30, 2), Math.toRadians(90))
                .addTemporalMarker(2, () -> {
                    // This marker runs two seconds into the trajectory
                    theRobot.setIntakeToBatteringRam();
                    // Run your action in here!
                })
                .build();

        Trajectory Trajectory2 = drive.trajectoryBuilder(Trajectory1.end())
                .strafeLeft(5)
                .build();


        Trajectory traj3 = drive.trajectoryBuilder(Trajectory2.end())
                .back(5)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .strafeLeft(10)
                .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj4.end())
                .splineTo(new Vector2d(32, 40), Math.toRadians(90))
                .build();

        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .strafeRight(18)
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj7.end())
                .forward(1)
                .build();

        if (isStopRequested()) return;

        drive.followTrajectory(Trajectory1);
        theRobot.placePurplePixel();
        //drive.followTrajectory(Trajectory2);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
       // drive.followTrajectory(traj6);
       // drive.followTrajectory(traj7);
       // drive.followTrajectory(traj5);

    }
}
