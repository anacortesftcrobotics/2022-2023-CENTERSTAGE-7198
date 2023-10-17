package org.firstinspires.ftc.teamcode.logancode;

public class PathMarker {

    private double x,y,r,vx,vy,vr;

    public PathMarker(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public PathMarker(double x, double y, double r, double vx, double vy, double vr) {
        this.x = x;
        this.y = y;
        this.r = r;
        this.vx= vx;
        this.vy= vy;
        this.vr= vr;

    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getR() {
        return r;
    }

    public double getVx() {
        return vx;
    }

    public double getVy() {
        return vy;
    }

    public double getVr() {
        return vr;
    }

    public String toString()
    {
        return "x: " + x + "y: " + y + "r: " + r + "vx: " + vx + "vy: " + vy + "vr: " + vr;
    }

    public double distance(PathMarker point)
    {
        double dist = Math.abs(Math.sqrt((x - point.x) * (y - point.y)));
        return dist;
    }
}
