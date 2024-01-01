package org.firstinspires.ftc.teamcode.odometry;

import org.firstinspires.ftc.teamcode.kinematics.Pose2D;

/**
 * This Odometry class provides tracking for an FTC Robot on the field.
 *
 * @author      Kai Wallis
 * @version     %I%, %G%
 * @deprecated
 */
public class Odo5 implements Odometry {

    private Pose2D fieldPose;
    private Pose2D deltaPose = new Pose2D();

    private final double diagonal; //distance between the center of rotation and a forward encoder.
    private final double rotationalConstant;
    // rotationalConstant is a constant which when multiplied with forward encoder values gives an approximated
    // distance traveled along an arc with the focus as the center of rotation. Value assumes movement is
    // purely rotational and non-rotational movement is canceled out by other encoders.
    private final double centerEncoderOffset;
    private final double distancePerTick;

    /**
     * Class constructor using custom encoder distances & dimensions.
     *
     * @param distanceLtoR          the distance between the left and right encoders.
     * @param centerEncoderOffset   the distance the center encoder is behind the center of rotation.
     * @param frontEncodersOffset   the distance the left & right encoders are forward of the center of rotation.
     * @param encoderDiameter       the diameter of the encoder wheels.
     * @param ticksPerRev           the encoder's ticks per revolution.
     */
    public Odo5(double distanceLtoR, double frontEncodersOffset, double centerEncoderOffset,
                double encoderDiameter, int ticksPerRev) {
        this.fieldPose = new Pose2D();
        this.diagonal = Math.sqrt(frontEncodersOffset * frontEncodersOffset + distanceLtoR * distanceLtoR / 4.0);
        this.centerEncoderOffset = centerEncoderOffset;
        this.rotationalConstant = 1.0 / Math.cos(Math.atan(frontEncodersOffset / distanceLtoR * 2.0));
        this.distancePerTick = encoderDiameter * Math.PI / ticksPerRev;
    }

    /**
     * Class constructor using custom encoder distances & dimensions.
     *
     * @param startingPose          Pose2D object representing the object's current position & heading.
     * @param distanceLtoR          the distance between the left and right encoders.
     * @param centerEncoderOffset   the distance the center encoder is behind the center of rotation.
     * @param frontEncodersOffset   the distance the left & right encoders are forward of the center of rotation.
     * @param encoderDiameter       the diameter of the encoder wheels.
     * @param ticksPerRev           the encoder's ticks per revolution.
     */
    public Odo5(Pose2D startingPose, double distanceLtoR, double frontEncodersOffset, double centerEncoderOffset,
                double encoderDiameter, int ticksPerRev) {
        this.fieldPose = new Pose2D(startingPose);
        this.diagonal = Math.sqrt(frontEncodersOffset * frontEncodersOffset + distanceLtoR * distanceLtoR / 4.0);
        this.centerEncoderOffset = centerEncoderOffset;
        this.rotationalConstant = 1.0 / Math.cos(Math.atan(frontEncodersOffset / distanceLtoR * 2.0));
        this.distancePerTick = encoderDiameter * Math.PI / ticksPerRev;
    }

    /**
     * Updates the object's position on the field based on the change in encoder positions then returns the updated
     * field position.
     * @param leftEncoder   change in tick position of the left encoder, (+) when moving forward.
     * @param rightEncoder  change in tick position of the right encoder, (+) when moving forward.
     * @param centerEncoder change in tick position of the center encoder, (+) when strafing right.
     * @return              the object's current pose on the field.
     */
    public Pose2D update(int leftEncoder, int rightEncoder, int centerEncoder) {
        double left = distancePerTick * leftEncoder;
        double right = distancePerTick * rightEncoder;
        double center = distancePerTick * centerEncoder;

        double deltaHRad = (right - left) * rotationalConstant * 0.5 / diagonal;
        double forward = (left + right) / 2.0;
        double strafe = center - deltaHRad * centerEncoderOffset;

        Pose2D lastMove = new Pose2D(strafe, forward, deltaHRad);
//        deltaPose = lastMove.getRevolvedRad(fieldPose.getHeadingRad());
//        fieldPose.add(deltaPose);
        fieldPose.addRelativePose(lastMove);

        return fieldPose;
    }

    /**
     * Returns the object's current pose on the field.
     * @return  Pose2D object representing the object's current position & heading.
     */
    public Pose2D getFieldPose() {
        return new Pose2D(fieldPose);
    }

    /**
     * Returns the latest change in the object's pose on the field
     * @return  Pose2D object representing the object's change in position & heading.
     */
    public Pose2D getDeltaPose() {
        return new Pose2D(deltaPose);
    }

    /**
     * Resets the object's pose on the field to a specified pose.
     * @param newPose   Pose2D object representing the object's new position & heading.
     */
    public void setFieldPose(Pose2D newPose) {
        this.fieldPose = new Pose2D(newPose);
    }

    /**
     * Resets the object's pose on the field, with the x, y, & heading being equal to 0.0.
     */
    public void setFieldPose() {
        this.fieldPose = new Pose2D();
    }
}
