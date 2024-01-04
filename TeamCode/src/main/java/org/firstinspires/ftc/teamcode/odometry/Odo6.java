package org.firstinspires.ftc.teamcode.odometry;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.firstinspires.ftc.teamcode.kinematics.Pose2D;

/**
 * This Odometry class provides tracking for an FTC Robot on the field.
 * This class is based on the ThreeTrackingWheelLocalizer class in ACME Robotics' Roadrunner.
 *
 * @author      kaiwallis
 * @version     %I%, %G%
 */
public class Odo6 implements Odometry{
    private Pose2D fieldPose;
    private Pose2D deltaPose = new Pose2D();

    private final DecompositionSolver deltaPoseSolver;

    private final double distancePerTick;

    /**
     * Class constructor using custom encoder distances & dimensions.
     *
     * @param encoder1Pose      Pose2D object representing encoder 1's position & angle relative to the robot's center.
     * @param encoder2Pose      Pose2D object representing encoder 2's position & angle relative to the robot's center.
     * @param encoder3Pose      Pose2D object representing encoder 3's position & angle relative to the robot's center.
     * @param encoderDiameter       the diameter of the encoder wheels.
     * @param ticksPerRev           the encoder's ticks per revolution.
     */
    public Odo6(Pose2D encoder1Pose, Pose2D encoder2Pose, Pose2D encoder3Pose,
                double encoderDiameter, int ticksPerRev) {
        this.fieldPose = new Pose2D();

        Pose2D[] encoderPoses = new Pose2D[3];
        encoderPoses[0] = encoder1Pose;
        encoderPoses[1] = encoder2Pose;
        encoderPoses[2] = encoder3Pose;

        this.deltaPoseSolver = getDeltaPoseSolver(encoderPoses);

        this.distancePerTick = encoderDiameter * Math.PI / ticksPerRev;
    }

    /**
     * Class constructor using custom encoder poses and dimensions.
     *
     * @param startingPose      Pose2D object representing the object's current position & heading.
     * @param encoder1Pose      Pose2D object representing encoder 1's position & angle relative to the robot's center.
     * @param encoder2Pose      Pose2D object representing encoder 2's position & angle relative to the robot's center.
     * @param encoder3Pose      Pose2D object representing encoder 3's position & angle relative to the robot's center.
     * @param encoderDiameter   the radius of the encoder wheels.
     * @param ticksPerRev       the encoder's ticks per revolution.
     */
    public Odo6(Pose2D startingPose, Pose2D encoder1Pose, Pose2D encoder2Pose, Pose2D encoder3Pose,
                double encoderDiameter, int ticksPerRev) {
        this.fieldPose = new Pose2D(startingPose);

        Pose2D[] encoderPoses = new Pose2D[3];
        encoderPoses[0] = encoder1Pose;
        encoderPoses[1] = encoder2Pose;
        encoderPoses[2] = encoder3Pose;

        this.deltaPoseSolver = getDeltaPoseSolver(encoderPoses);

        this.distancePerTick = encoderDiameter * Math.PI / ticksPerRev;
    }

    /**
     * Creates a DecompositionSolver which converts the distance each encoder travels [e1, e2, e3]
     * to a vector representing relative change in pose [x, y, h].
     * Throws a runtime exception if the dead wheel configuration does not create a valid solver.
     *
     * @param encoderPoses  array of encoder poses relative to the robot's center. (In order.)
     * @return              solver for converting delta encoder distance to delta relative pose.
     */
    private DecompositionSolver getDeltaPoseSolver(Pose2D[] encoderPoses) {
        Array2DRowRealMatrix deltaEncoderstoDeltaPose = new Array2DRowRealMatrix(3, 3);
        for (int i = 0; i < 3; i++) {
            double eH = encoderPoses[i].getHeadingRad();
            deltaEncoderstoDeltaPose.setEntry(i, 0, Math.cos(eH)); // effect of x
            deltaEncoderstoDeltaPose.setEntry(i, 1, Math.sin(eH)); // effect of y
            deltaEncoderstoDeltaPose.setEntry(i, 2,
                    encoderPoses[i].getX() * Math.sin(eH) - encoderPoses[i].getY() * Math.cos(eH)
            );
        }
        DecompositionSolver deltaPoseSolver = (new LUDecomposition(deltaEncoderstoDeltaPose)).getSolver();

        if(!deltaPoseSolver.isNonSingular()) {
            throw new RuntimeException("Current wheel arrangement is invalid.");
        }

        return deltaPoseSolver;
    }

    /**
     * Updates the object's position on the field based on the change in encoder positions then returns the updated
     * field position.
     * @param encoder1      change in tick position of the first encoder,
     *                      (+) when traveling towards the (+) x-axis if encoder1Pose heading is 0.
     * @param encoder2      change in tick position of the second encoder,
     *                      (+) when traveling towards the (+) x-axis if encoder1Pose heading is 0.
     * @param encoder3      change in tick position of the third encoder,
     *                      (+) when traveling towards the (+) x-axis if encoder1Pose heading is 0.
     * @return              the object's current pose on the field.
     */
    public Pose2D update(int encoder1, int encoder2, int encoder3) {
        double[] deltaDistances = new double[3];
        deltaDistances[0] = encoder1 * distancePerTick;
        deltaDistances[1] = encoder2 * distancePerTick;
        deltaDistances[2] = encoder3 * distancePerTick;

        Pose2D deltaRelativePose = getDeltaRelativePose(deltaDistances);
        this.deltaPose = deltaRelativePose.getRevolvedRad(fieldPose.getHeadingRad());
        this.fieldPose.addRelativePose(deltaRelativePose);
        return fieldPose;
    }

    /**
     * Returns the change in pose relative to the robot's pose given an array of distances the encoders traveled.
     * @param deltaDistances    array of distances traveled by the encoders, in order.
     * @return                  Pose2D representing the object's relative change in position.
     */
    private Pose2D getDeltaRelativePose(double[] deltaDistances) {
        RealMatrix encoders = new Array2DRowRealMatrix(deltaDistances);
        RealMatrix solved = this.deltaPoseSolver.solve(encoders); // used by roadrunner. Needs testing.

         return new Pose2D(
                solved.getEntry(0, 0),
                solved.getEntry(1, 0),
                solved.getEntry(2, 0)
        );
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

