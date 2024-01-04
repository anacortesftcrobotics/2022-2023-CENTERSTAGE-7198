package org.firstinspires.ftc.teamcode.kinematics;

public class Pose2D extends Vector2D {
    private double headingRad;

    /**
     * Creates a default Pose2D object, the position being at the origin with a heading of 0.
     */
    public Pose2D() {
        super();
        this.headingRad = 0.0;
    }

    /**
     * Creates a Pose2D object based on x, y, and heading values.
     * @param x             x coordinate.
     * @param y             y coordinate.
     * @param headingRad    heading in radians, (+) being counterclockwise.
     */
    public Pose2D(double x, double y, double headingRad) {
        super(x, y);
        this.headingRad = headingRad;
    }

    /**
     * Creates a Pose2D object based on a vector object and heading.
     * @param vector        vector object representing x & y coordinates.
     * @param headingRad    heading in radians, (+) being counterclockwise.
     */
    public Pose2D(Vector2D vector, double headingRad) {
        super(vector.getX(), vector.getY());
        this.headingRad = headingRad;
    }

    /**
     * Creates a Pose2D object equal to another Pose2D object.
     * @param pose      Pose2D object to copy values from.
     */
    public Pose2D(Pose2D pose) {
        super(pose.getX(), pose.getY());
        this.headingRad = pose.getHeadingRad();
    }

    /**
     * Returns the heading in radians.
     * @return  heading in radians.
     */
    public double getHeadingRad() {
        return headingRad;
    }

    /**
     * Adds deltaHeadingRad to the heading of this Pose2D object.
     * @param deltaHeadingRad   heading to add in radians, (+) being counterclockwise.
     */
    public void turnRad(double deltaHeadingRad) {
        this.headingRad += deltaHeadingRad;
    }

    /**
     * Returns the sum of two Pose2D objects, summing position and heading. (Does not modify objects.)
     * @param otherPose     another Pose2D object to sum with this object.
     * @return              a new Pose2D object representing the sum of the two Pose2D objects
     */
    public Pose2D plus(Pose2D otherPose) {
        return new Pose2D(super.plus(otherPose.getVector()), headingRad + otherPose.headingRad);
    }

    /**
     * Returns the differance of two Pose2D objects. (Does not modify objects.)
     * @param otherPose     another Pose2D object which is subtracted from this object.
     * @return              a new Pose2D object representing the differance of the two Pose2D objects
     */
    public Pose2D minus(Pose2D otherPose) {
        return new Pose2D(super.minus(otherPose.getVector()), headingRad - otherPose.headingRad);
    }

    /**
     * Adds otherPose to this (modifies this).
     * @param otherPose     other Pose2D object.
     */
    public void add(Pose2D otherPose) {
        super.add(otherPose.getVector());
        this.headingRad += otherPose.headingRad;
    }

    /**
     * Subtracts otherPose from this (modifies this).
     * @param otherPose     other Pose2D object.
     */
    public void subtract(Pose2D otherPose) {
        super.subtract(otherPose.getVector());
        this.headingRad -= otherPose.headingRad;
    }

    /**
     * Returns a Pose2D object equivalent to this object if it was rotated around the origin angleRad degrees.
     * @param angleRad      degrees to rotate, (+) being counterclockwise.
     * @return              a new Pose2D object representing this object rotated around the origin
     */
    @Override
    public Pose2D getRevolvedRad(double angleRad) {
        return new Pose2D(super.getRevolvedRad(angleRad), headingRad + angleRad);
    }

    /**
     * Rotates the Pose2D object around the origin angleRad degrees.
     * @param angleRad      degrees to rotate, (+) being counterclockwise.
     */
    @Override
    public void revolveRad(double angleRad) {
        super.revolveRad(angleRad);
        turnRad(angleRad);
    }

    /**
     * Returns a Pose2D object equivalent to relativePose added to this,
     * with relativePose having an origin at this object's pose.
     * @param relativePose      a Pose relative to this object's pose, with an origin in line with this object's pose.
     */
    public void addRelativePose(Pose2D relativePose) {
        this.add((Vector2D) relativePose.getRevolvedRad(this.headingRad));
        this.headingRad += relativePose.headingRad;
    }

    /**
     * Adds a Pose2D object to this, with relativePose having an origin at this object's pose.
     * @param relativePose      a Pose relative to this object's pose, with an origin in line with this object's pose.
     * @return                  a new Pose2D object representing this object rotated around the origin.
     */
    public Pose2D plusRelativePose(Pose2D relativePose) {
        return new Pose2D(
                super.plus(relativePose.getRevolvedRad(this.headingRad)),
                this.headingRad + relativePose.headingRad);
    }

    /**
     * Indicates whether some other object is "equal to" this one.
     * @param o     the reference object with which to compare
     * @return      true if this object is the same as the obj argument; false otherwise.
     */
    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        if (!super.equals(o)) return false;
        Pose2D pose2D = (Pose2D) o;
        return Double.compare(pose2D.headingRad, headingRad) == 0;
    }

    /**
     * Returns a string representation of the object.
     * @return  a string representation of the object.
     */
    @Override
    public String toString() {
        return this.getClass().getSimpleName() +
                "{x=" + super.getX() +
                ", y=" + super.getY() +
                ", headingRad= " + headingRad + "}";
    }
}
