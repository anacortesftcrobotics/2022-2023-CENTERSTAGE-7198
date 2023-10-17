package org.firstinspires.ftc.teamcode.logancode;

public class Position2D {

    private double x;
    private double y;

    public Position2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public String toString()
    {
        return x + " " + y;
    }

    public double distance(Position2D point)
    {
        double dist = Math.abs(Math.sqrt((x - point.x) * (y - point.y)));
        return dist;
    }
}
