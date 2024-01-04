package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.kinematics.Pose2D;

/**
 * Feeds data from encoders to an Odometry class.
 * @author      kaiwallis
 * @version     %I%, %G%
 */
public class _7198_OdoController {
    public _7198_OdoController() {}

    // facing y-axis
    Odometry odo = new Odo6(
            new Pose2D(-19.64, 2.37, Math.toRadians(90)),
            new Pose2D(19.64, 2.37, Math.toRadians(90)),
            new Pose2D(0, -14.93, Math.toRadians(180)),
            5,
            8192
    );

    // facing x-axis
//    Odometry odo = new Odo6(
//            new Pose2D(2.37, 19.64, Math.toRadians(0)),
//            new Pose2D(2.37, -19.64, Math.toRadians(0)),
//            new Pose2D(-14.93, 0, Math.toRadians(90)),
//            5,
//            8192
//    );

    private DcMotor encoderLeft, encoderRight, encoderBack;

    private int encoder1, encoder2, encoder3;
    private int lastEncoder1, lastEncoder2, lastEncoder3;

    /**
     * Initializes the three odometry wheels
     * In this case, it's the four drive motors used by the robot.
     */
    public void initializeHardware (HardwareMap hardwareMap)
    {
        encoderLeft = hardwareMap.get(DcMotor.class, "backLeft");
        encoderRight = hardwareMap.get(DcMotor.class, "frontLeft");
        encoderBack = hardwareMap.get(DcMotor.class, "frontRight");
    }

    /**
     * Updates odometry.
     */
    public void update() {
        updateEncoders();
        odo.update(
                encoder1 - lastEncoder1,
                encoder2 - lastEncoder2,
                encoder3 - lastEncoder3
        );
    }

    public Odometry acessOdo6()
    {
        return odo;
    }

    /**
     * Updates the values stored representing encoder postions.
     */
    private void updateEncoders() {
        lastEncoder1 = encoder1;
        lastEncoder2 = encoder2;
        lastEncoder3 = encoder3;

        encoder1 = encoderLeft.getCurrentPosition();
        encoder2 = encoderRight.getCurrentPosition();
        encoder3 = encoderBack.getCurrentPosition();
    }

    /**
     * Returns the robot's current pose on the field.
     * @return  Pose2D object representing the robot's current position & heading.
     */
    public Pose2D getFieldPose() {
        return odo.getFieldPose();
    }

    /**
     * Returns the latest change in the robot's pose on the field.
     * @return  Pose2D object representing the robot's change in position & heading.
     */
    public Pose2D getDeltaPose() {
        return odo.getDeltaPose();
    }

    /**
     * Returns the robot's x position on the field.
     * @return  x position on the field.
     */
    public double getX() {
        return odo.getFieldPose().getX();
    }

    /**
     * Returns the robot's y position on the field.
     * @return  y position on the field.
     */
    public double getY() {
        return odo.getFieldPose().getY();
    }

    /**
     * Returns the robot's heading on the field in radians.
     * @return  heading on the field in radians.
     */
    public double getHeadingRad() {
        return odo.getFieldPose().getHeadingRad();
    }

    /**
     * Returns the robot's heading on the field in degrees.
     * @return  heading on the field in degrees.
     */
    public double getHeadingDeg() {
        return Math.toDegrees(odo.getFieldPose().getHeadingRad());
    }
}
