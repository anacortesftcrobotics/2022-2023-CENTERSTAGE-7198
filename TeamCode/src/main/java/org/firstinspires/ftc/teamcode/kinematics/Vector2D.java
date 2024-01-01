package org.firstinspires.ftc.teamcode.kinematics;

import java.util.Objects;

/**
 * This class represents a 2-dimensional vector.
 * @author      Kai Wallis
 * @version     %I%, %G%
 */
public class Vector2D {

    private double x;
    private double y;

    public Vector2D() {
        this.x = 0.0;
        this.y = 0.0;
    }

    /**
     * Constructor creating a vector using x & y coordinates.
     * @param x     x coordinate.
     * @param y     y coordinate.
     */
    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Returns the vector's x value.
     * @return      vector's x value.
     */
    public double getX() {
        return x;
    }

    /**
     * Returns the vector's y value.
     * @return      vector's y value.
     */
    public double getY() {
        return y;
    }

    /**
     * Returns a double array representing the x & y dimensions of the vector
     * @return      double[] being equal to {x, y}
     */
    public double[] getCoords() {
        return new double[] {x, y};
    }

    /**
     * Changes the x dimension of the vector.
     * @param x     new x value.
     */
    public void setX(double x) {
        this.x = x;
    }

    /**
     * Changes the y dimension of the vector.
     * @param y     new y value.
     */
    public void setY(double y) {
        this.y = y;
    }

    /**
     * Changes the x & y dimensions of the vector
     * @param x     new x value.
     * @param y     new y value.
     */
    public void setCoords(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Returns the vector's magnitude.
     * @return      vector's magnitude
     */
    public double getMagnitude() {
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Returns the vector's angle from the positive x-axis;
     * @return      angle from the positive x-axis.
     */
    public double getAngleRad() {
        return Math.atan(y/x);
    }

    /**
     * Returns the sum of Vector2D and other Vector.
     * @param otherVector   another Vector2D object.
     * @return              sum of Vector2D and otherVector.
     */
    public Vector2D plus(Vector2D otherVector) {
        return new Vector2D(this.x + otherVector.x, this.y + otherVector.y);
    }

    /**
     * Returns the differance of Vector2D and otherVector.
     * @param otherVector   another Vector2D object.
     * @return              differance of Vector2D and otherVector.
     */
    public Vector2D minus(Vector2D otherVector) {
        return new Vector2D(this.x - otherVector.x, this.y - otherVector.y);
    }

    /**
     * Adds otherVector to vector2D.
     * @param otherVector   another Vector2D object.
     */
    public void add(Vector2D otherVector) {
        this.x += otherVector.getX();
        this.y += otherVector.getY();
    }

    /**
     * Subtracts otherVector from vector2D.
     * @param otherVector   another Vector2D object.
     */
    public void subtract(Vector2D otherVector) {
        this.x -= otherVector.getX();
        this.y -= otherVector.getY();
    }

    /**
     * returns a Vector2D object rotated angleRad radians around the origin.
     * @param angleRad      radians to rotate by.
     * @return              Vector2D rotated angleRad radians.
     */
    public Vector2D getRevolvedRad(double angleRad) {
        double localX = x * Math.cos(angleRad) - y * Math.sin(angleRad);
        double localY = x * Math.sin(angleRad) + y * Math.cos(angleRad);

        return new Vector2D(localX, localY);
    }

    /**
     * Rotates Vector2D angleRad radians around the origin.
     * @param angleRad      radians to rotate by.
     */
    public void revolveRad(double angleRad) {
        double localX = x * Math.cos(angleRad) - y * Math.sin(angleRad);
        double localY = x * Math.sin(angleRad) + y * Math.cos(angleRad);

        this.x = localX;
        this.y = localY;
    }

    /**
     * Returns this Vector2D object
     * @return      this Vector2D.
     */
    public Vector2D getVector() {
        return this;
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
        Vector2D vector2D = (Vector2D) o;
        return Double.compare(vector2D.x, x) == 0 && Double.compare(vector2D.y, y) == 0;
    }

    /**
     * Returns a string representation of the object.
     * @return  a string representation of the object.
     */
    @Override
    public String toString() {
        return this.getClass().getSimpleName() +
                "{" +
                "x=" + x +
                ", y=" + y +
                '}';
    }
}
