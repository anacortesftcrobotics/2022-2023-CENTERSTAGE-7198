package org.firstinspires.ftc.teamcode.logancode;

/**A Utility class created by Logan Rainchild for FTC robotics 2023-present
 * Best wishes and happy programming!
 */
public class LogsUtils {

    /**rounds the input to the p decimal place.
     *
     * @param input value
     * @param p decimal position
     * @return rounded value
     */
    public static double roundBetter(double input, double p)
    {
        return Math.round(input * Math.pow(10,p)) / Math.pow(10,p);
    }

    /**
     * remaps input so small values near 0 are 0 and values above a threshold are valued.
     * The function will reach 1 when double input is 1.
     *
     * @param input the input to be mapped
     * @param a a non-negative real number
     */
    public static double deadZone(double input,double a) {
        return Math.max(input*(1+a),a)+Math.min(input*(1+a),-a);
    }

    /**
     * Maps input to a exponential equation keeping its sign. (calculated in desmos)
     * @param input the input to be mapped
     * @param exponent the power of the equation
     */
    public static double exponentialRemapAnalog(double input, double exponent) {
        return Math.min(Math.pow(Math.max(input,0),exponent),1) + Math.max(-Math.pow(Math.min(input,0),exponent),-1);
    }

    /**
     * Maps input to a exponential equation keeping its sign. (calculated in desmos)
     * @param input the input to be clamped
     * @param min the minimum value (inclusive)
     * @param max the maximum value (inclusive)
     */
    public static double clamp(double input, double min, double max)
    {
        return Math.max(Math.min(input,max),min);
    }

    /**
     * Maps input to a exponential equation keeping its sign. (calculated in desmos)
     * @param input the input to be clamped
     * @param min the minimum value (inclusive)
     * @param max the maximum value (inclusive)
     */
    public static int clamp(int input, int min, int max)
    {
        return Math.max(Math.min(input,max),min);
    }
}
