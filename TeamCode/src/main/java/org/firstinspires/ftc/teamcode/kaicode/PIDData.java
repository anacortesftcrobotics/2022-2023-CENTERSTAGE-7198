package org.firstinspires.ftc.teamcode.kaicode;

/**
 * This class calculates and outputs corrections against an error value and records data for analysis.
 * @author      Kai Wallis
 * @version     %I%, %G%
 */
public class PIDData {
    private double kP;
    private double kI;
    private double kD;

    private double devaluePastI = 1.0; //Devalues old values of the integral. Range: 1.0-0.0.

    private double lastE; //last error value
    private double integral; //integral of all recorded errors

    public String log = "Time(ms) Loops Error Proportional Integral Derivative/n"; //initializes log of PID
    private int loops;

    /**
     * Class constructor with a Proportional, Integral, and Derivative based corrections,
     * plus a setting for devaluing old integral values instead of considering them fully.
     * @param proportionalConstant  the applied coefficient for the proportional corrections.
     * @param integralConstant      the applied coefficient for the integral corrections.
     * @param derivativeConstant    the applied coefficient for the derivative corrections.
     * @param devalue               modifier to old integral values (range 1.0-0). Try ~0.66 if using, 1 default.
     */
    public PIDData(double proportionalConstant, double integralConstant, double derivativeConstant, double devalue) {
        kP = proportionalConstant;
        kI = integralConstant;
        kD = derivativeConstant;
        devaluePastI = devalue;
    }

    /**
     * Class constructor with a Proportional, Integral, and Derivative based corrections.
     * @param proportionalConstant  the applied coefficient for the proportional corrections.
     * @param integralConstant      the applied coefficient for the integral corrections.
     * @param derivativeConstant    the applied coefficient for the derivative corrections.
     */
    public PIDData(double proportionalConstant, double integralConstant, double derivativeConstant) {
        kP = proportionalConstant;
        kI = integralConstant;
        kD = derivativeConstant;
    }

    /**
     * Class constructor with a Proportional and Derivative based corrections.
     * @param proportionalConstant  the applied coefficient for the proportional corrections.
     * @param derivativeConstant    the applied coefficient for the derivative corrections.
     */
    public PIDData(double proportionalConstant, double derivativeConstant) {
        kP = proportionalConstant;
        kI = 0;
        kD = derivativeConstant;
    }

    /**
     * Class constructor with a Proportional based correction.
     * @param proportionalConstant  the applied coefficient for the proportional corrections.
     */
    public PIDData(double proportionalConstant) {
        kP = proportionalConstant;
        kI = 0;
        kD = 0;
    }

    /**
     * Class constructor disabling P, I, and D corrections.
     * Output is always 0, use to disable output of class.
     */
    public PIDData() {
        kP = 0;
        kI = 0;
        kD = 0;
    }

    /**
     * Returns the last error value .
     * @return      the last error value.
     */
    public double getLastE() {
        return lastE;
    }

    /**
     * Returns the running sum (integral) of the error.
     * @return      integral of the error.
     */
    public double getI() {
        return integral;
    }

    /**
     * returns derivative between an input error and the last error recorded in the method.
     * @param error     latest error value.
     * @return          derivative between current and last error.
     */
    public double getD(double error) {
        return error-lastE;
    }

    /**
     * Returns a control signal to correct the actual value based on an expected and actual value.
     * @param expectedValue     the expected value (such as the expected position) the parameter should have.
     * @param actualValue       the actual value (such as the current position) the parameter should have.
     * @return                  the suggested corrective action.
     */
    public double getPID(double expectedValue, double actualValue) {
        double e;
        double u;
        
        loops += 1;
        log += System.currentTimeMillis() + " " + loops + " "; //logs system time & loop #
        
        e = expectedValue - actualValue; //r-y
        log += e + " "; //logs error

        u = kP * e;
        log += u + " "; //logs P
        setI(e);
        u += kI * getI();
        log += kI * getI() + " "; // logs I

        u += kD * getD(e);
        log += kD * getD(e) + "/n"; //logs D

        lastE = e; //stores the newer error value as the older. (at end of method)
        return u;
    }

    /**
     * Returns a control signal to correct the actual value based on an error value.
     * @param error     the error between the expected value and actual value.
     * @return          the suggested corrective action.
     */
    public double getPID(double error) {
        double u;
        
        loops += 1;
        log += System.currentTimeMillis() + " " + loops + " "; //logs system time & loop #

        log += error + " "; //logs error

        u = kP * error;
        log += u + " "; //logs P

        setI(error);
        u += kI * getI();
        log += kI * getI() + " "; // logs I

        u += kD * getD(error);
        log += kD * getD(error) + "/n"; //logs D

        lastE = error; //stores the newer error value as the older. (at end of method)
        return u;
    }

    /**
     * Updates the integral to include latest error value.
     * @param error     most recent error value    
     */
    private void setI(double error) {
        integral = devaluePastI*integral + 0.5*(lastE + error); //0.5*(lastE-error) + error simplified + last integral value
    }

    //returns string of recorded log
    public String getLog()
    {
        return log;
    }
}