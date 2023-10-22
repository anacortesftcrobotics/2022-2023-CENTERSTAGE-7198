package org.firstinspires.ftc.teamcode;

/**
 * This class provides odometry tracking for an FTC Robot
 * @author      Kai Wallis
 * @version     %I%, %G%
 */
public class Odo1 {

    private double x = 0;     //x Position on field in cm
    private double y = 0;     //y Position on field in cm
    private double hRad = 0;  //heading angle on field in degrees, 0 is straight relative to start

    private double[] encodersLast = {0.0,0.0,0.0};  //left, right, center
    private double[] encoders = {0.0,0.0,0.0};      //l,r,c
    private double[] deltaEncoders = {0.0,0.0,0.0}; //l,r,c

    private double deltaX;
    private double deltaY;
    private double deltaHRad;

    private double distanceLtoR = 11.0;
    private double forwardOffset = 28.5;
    private double encoderDiameter = 4.9;
    private int ticksPerRev = 8192;
    private double cmPerTick = encoderDiameter * Math.PI/ticksPerRev;


    /**
     * Class constructor using default encoder distances & dimensions.
     */
    public Odo1() {}

    /**
     * Class constructor using custom encoder distances & dimensions.
     * @param disLtoR       the distance between left and right encoders in cm.
     * @param disMidtoC     the distance between midpoint of the left & right encoders and center encoder in cm. Should be (-) if in front.
     * @param diameter      the diameter of encoder wheels in cm.
     * @param ticksPerRevolution    the encoder ticks per revolution.
     */
    public Odo1(double disLtoR, double disMidtoC, double diameter, int ticksPerRevolution) {
        distanceLtoR = disLtoR;
        forwardOffset = disMidtoC;
        encoderDiameter = diameter;
        ticksPerRev = ticksPerRevolution;
        cmPerTick = encoderDiameter * Math.PI/ticksPerRev;
    }

    /**
     * Class constructor using the encoder outputs after being spun counterclockwise 10 times. 
     *      If not in range, input negative values into the code when running. 
     * @param rightEncoder      (+) ticks traveled by the right encoder.
     * @param leftEncoder       (-) ticks traveled by the left encoder.
     * @param centerEncoder     (+/-) ticks traveled by the center encoder.
     * @param diameter          the diameter of the encoder wheels in cm.
     * @param ticksPerRevolution    the encoder ticks per revolution.
     */
    public Odo1(int leftEncoder, int rightEncoder, int centerEncoder, double diameter, int ticksPerRevolution) {
        encoderDiameter = diameter;
        ticksPerRev = ticksPerRevolution;
        cmPerTick = encoderDiameter * Math.PI/ticksPerRev;

        distanceLtoR = Math.abs(cmFromCenter(rightEncoder, cmPerTick))
            + Math.abs(cmFromCenter(leftEncoder, cmPerTick));
        
        forwardOffset = cmFromCenter(centerEncoder, ticksPerRevolution);
    }

    private static double cmFromCenter(int ticks, double cmPerTick) {
        return ticks/10.0/cmPerTick/2.0/Math.PI;
    }

    /**
     * Returns the most recent change in the x dimension between the last 2 setEncoderPos.
     * @return  the x dimension change.
     */
    public double getDeltaX() {
        return deltaX;
    }

    /**
     * Returns the most recent change in the y dimension between the last 2 setEncoderPos.
     * @return  the y dimension change.
     */
    public double getDeltaY() {
        return deltaY;
    }

    /**
     * Returns the distance traveled between the last 2 setEncoderPos.
     * @return      the robot's current heading in degrees
     */
    public double getDeltaDistance() {
        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    }

    /**
     * Returns the robot's heading change in radians between the last 2 setEncoderPos.
     * @return      the robot's heading change in radians.
     */
    public double getDeltaHRad() {
        return deltaHRad;
    }

    /**
     * Returns the robot's heading change in degrees between the last 2 setEncoderPos.
     * @return      the robot's heading change in degrees
     */
   public double getDeltaHDeg() {
        return Math.toDegrees(deltaHRad);
    }

    /**
     * Returns the robot's current x position
     * @return      the robot's current x position
     */
    public double getX() {
        return x;
    }

    /**
     * Returns the robot's current y position
     * @return      the robot's current y position
     */
    public double getY() {
        return y;
    }

    /**
     * Returns the robot's current heading in radians
     * @return      the robot's current heading in radians
     */
    public double getHRad() {
        return hRad;
    }

    /**
     * Returns the robot's current heading in degrees
     * @return      the robot's current heading in degrees
     */
    public double getHDeg() {
        return Math.toDegrees(hRad);
    }

    /**
     * Returns the angle traveled between the last 2 setEncoderPos in radians.
     * @return      the angle traveled between the last 2 setEncoderPos in radians.
     */
    public double getVectorAngleRad() {
        return Math.atan(deltaY/deltaX);
    }

    /**
     * Returns the angle traveled between the last 2 setEncoderPos in degrees.
     * @return      the angle traveled between the last 2 setEncoderPos in degrees.
     */
    public double getVectorAngleDeg() {
        return Math.toDegrees(Math.atan(deltaY/deltaX));
    }

    /**
     * Resets encoder history & delta values.
     */
    public void reset() {
        for (int i = 2; i >= 0; i--) {
            encodersLast[i] = 0.0;
            encoders[i] = 0.0;
            deltaEncoders[i] = 0.0;
        }

        deltaX = 0.0;
        deltaY = 0.0;
        deltaHRad = 0.0;
    }

    /**
     * Resets field coords, encoder history & delta values.
     */
    public void resetAll() {
        reset();

        x = 0;
        y = 0;
        hRad = 0;
    }

    /**
     * Sets the current position & heading on a coordinate grid.
     * @param newX     new position on the x axis.
     * @param newY     new position on the y axis.
     * @param newHDeg  new heading in degrees.
     */
    public void setCoords(double newX, double newY, double newHDeg) {
        x = newX;
        y = newY;
        hRad = newHDeg * Math.PI/180;
    }

    /**
     * Updates the object based on updated encoder positions.
     * @param eLeft     new tick position of the left encoder, (+) when moving forward.
     * @param eRight    new tick position of the right encoder, (+) when moving forward.
     * @param eCenter   new tick position of the center encoder, (+) when strafing right.
     */
    public void setEncoderPos(int eLeft, int eRight, int eCenter) {
        encodersLast = encoders;

        encoders[0] = cmPerTick * eLeft;
        encoders[1] = cmPerTick * eRight;
        encoders[2] = cmPerTick * eCenter;
 
        for (int i = 2; i >= 0; i--)
            deltaEncoders[i] =  encoders[i] - encodersLast[i];
        
        deltaHRad = (deltaEncoders[0] - deltaEncoders[1]) / distanceLtoR;
        double deltaF = (deltaEncoders[1] + deltaEncoders[0]) / 2.0;
        double deltaR = deltaEncoders[2] - forwardOffset * deltaHRad;

        deltaX = deltaF * Math.cos(hRad) - deltaR * Math.sin(hRad);
        deltaY = deltaF * Math.sin(hRad) + deltaR * Math.cos(hRad);

        x += deltaX;
        y += deltaY;
        hRad += deltaHRad;
    }
}