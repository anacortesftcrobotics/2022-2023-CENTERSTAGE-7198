package org.firstinspires.ftc.teamcode.odometry;

import org.firstinspires.ftc.teamcode.kinematics.Pose2D;

/**
 * This interface defines the methods an Odometry class needs.
 * @author      Kai Wallis
 * @version     %I%, %G%
 */
public interface Odometry{
    /**
     * Updates the object's position on the field based on the change in encoder positions then returns the updated
     * field position.
     * @param encoder1      change in tick position of the first encoder.
     * @param encoder2      change in tick position of the second encoder.
     * @param encoder3      change in tick position of the third encoder.
     * @return              the object's current pose on the field.
     */
    public Pose2D update(int encoder1, int encoder2, int encoder3);

    /**
     * Returns the object's current pose on the field.
     * @return  Pose2D object representing the object's current position & heading.
     */
    public Pose2D getFieldPose();

    /**
     * Returns the latest change in the object's pose on the field
     * @return  Pose2D object representing the object's change in position & heading.
     */
    public Pose2D getDeltaPose();

    /**
     * Resets the object's pose on the field to a specified pose.
     * @param newPose   Pose2D object representing the object's new position & heading.
     */
    public void setFieldPose(Pose2D newPose);

    /**
     * Resets the object's pose on the field, with the x, y, & heading being equal to 0.0.
     */
    public void setFieldPose();
}
