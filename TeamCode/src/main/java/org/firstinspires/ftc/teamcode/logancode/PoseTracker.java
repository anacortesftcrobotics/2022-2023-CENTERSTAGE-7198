package org.firstinspires.ftc.teamcode.logancode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.robot.Robot;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

// This is a class that provides useful methods using the REV IMU
public class PoseTracker {

    // Define robot object
    _7198CSRobot r;

    // Define IMU object
    IMU imu;

    // Define necessary fields
    Orientation lastAngles;
    double globalAngle, correction;

    // Constructs a PoseTracker object
    public PoseTracker(IMU imu) {
        this.imu = imu;
    }

    private Orientation getAngles() {

        return imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
    }

    // Reset current tracked angle, and store the current true heading
    public void resetAngle() {
        lastAngles = getAngles();
        globalAngle = 0;
    }

    // Calculate and return the current tracked angle using the previous true angle and the current true angle
    public double getAngle() {
        Orientation angles = getAngles();
        double deltaAngle = 0;
        if (angles != null) {
            deltaAngle = angles.thirdAngle - lastAngles.thirdAngle;
        } else {
            //r.telemetry.addLine("Err0r");
            //r.telemetry.update();
        }

        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    // Returns the amount to correct a straight movement to avoid slight power imbalance
    public double checkDirection() {
        double correction, angle, gain = .1;

        angle = getAngle();

        if (angle == 0) {
            correction = 0;
        } else {
            correction = -angle;
        }

        correction = correction * gain;

        return correction;
    }
}
