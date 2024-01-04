package org.firstinspires.ftc.teamcode.kinematics;

/**
 * This class represents a 2-dimensional vector.
 * @author      Logan Rainchild
 * @version     %I%, %G%
 */
public class PoseVelocity2D extends Pose2D{

    private double Vx, Vy, VheadingRad;

    public PoseVelocity2D() {
        super();
    }

    public PoseVelocity2D(double x, double y, double headingRad, double Vx, double Vy, double VheadingRad) {
        super(x, y, headingRad);
        this.Vx = Vx;
        this.Vy = Vy;
        this.VheadingRad = VheadingRad;
    }

    public double getVx() {
        return Vx;
    }

    public double getVy() {
        return Vy;
    }

    public double getVheadingRad() {
        return VheadingRad;
    }

    public String serialize(){
            return "x" + super.getX() + "; y" + super.getY() + "; r" + super.getHeadingRad() + "; vx" + Vx + "; vy" + Vy + "; vr" + VheadingRad + ";";
    }

    public double distance(PoseVelocity2D poseV)
    {
        double dist = Math.sqrt(Math.pow(super.getX() - poseV.getX(),2) + Math.pow(super.getX() - poseV.getY(),2));
        return dist;
    }
}
