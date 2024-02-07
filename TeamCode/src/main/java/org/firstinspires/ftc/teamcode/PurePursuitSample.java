package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.*;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.

@Autonomous(name="CS 8934 Pure Pursuit Sample", group="CenterStage")
public class PurePursuitSample extends LinearOpMode {

    // define our constants
    private PurePursuitCommand ppCommand;
    private Motor frontLeft, frontRight, backLeft, backRight;
    private DcMotor leftEncoder, rightEncoder, centerEncoder;
    private OdoControllerAlfalfa kaiOdo;
    private CenterStageRobot robot;

    @Override
    public void runOpMode() {

        kaiOdo = new OdoControllerAlfalfa();
        kaiOdo.initializeHardware(hardwareMap);
        robot = new CenterStageRobot(hardwareMap);
        waitForStart();

       while (opModeIsActive()) {

        Waypoint p1 = new StartWaypoint(0,0);
        Waypoint p2 = new GeneralWaypoint(0,150,10,0.3,0,50);
        Waypoint p3 = new EndWaypoint(50,125,0,0.3,0,50,3000,2005);

        Path m_path = new Path(p1, p2, p3);
        m_path.init();
           while (!m_path.isFinished() && opModeIsActive()) {
               if (m_path.timedOut()) {
                   telemetry.addLine("timed out");
               }

               // return the motor speeds
               telemetry.addData("x", kaiOdo.getX());
               telemetry.addData("y", kaiOdo.getY());
               telemetry.addData("heading", kaiOdo.getHeadingDeg());
               kaiOdo.update();
               telemetry.update();
               double speeds[] = m_path.loop(kaiOdo.getX(), kaiOdo.getY(),
                       kaiOdo.getHeadingDeg());

               robot.mecanumX(speeds[0], speeds[1], speeds[2]);
               kaiOdo.update();
               kaiOdo.getFieldPose();
           }
           robot.halt();
           if (m_path.isFinished()){
               telemetry.addLine("done!");
           }
        }
    }
}
