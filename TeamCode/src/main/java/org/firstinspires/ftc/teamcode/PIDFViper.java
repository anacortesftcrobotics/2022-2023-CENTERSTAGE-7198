package org.firstinspires.ftc.teamcode;

public class PIDFViper extends PIDFController{
    private double correction;
    private double kG;
    private double kM;
    private double km;

    /**
     * Class constructor with Proportional, Integral, and Derivative based corrections,
     * plus a setting for devaluing old integral values instead of considering them fully.
     * Use launch() before using update().
     *
     * @param kP coefficient for proportional gain
     * @param kI coefficient for integral gain
     * @param kD coefficient for derivative gain
     * @param kV coefficient for applying velocity feedforward
     * @param kA coefficient for applying acceleration feedforward
     */
    public PIDFViper(double kP, double kI, double kD, double kV, double kA) {
        super(kP, kI, kD, kV, kA);
    }

    /**
     * kP coefficient for P
     * kI coefficient for I
     * kD coefficient for D
     * kV coefficient for Velocity
     * kA coefficient for Acceleration
     * kG coefficient for force of gravity upon arm
     * kM coefficient of coefficient for friction
     * km coeffffffficient for slope of gravity effect
     **/

    /*public PIDFArmControllerF(double kP, double kI, double kD, double kV, double kA, double kG, double kM) {
        super(kP, kI, kD, kV, kA);
        this.kG = kG;
        this.kM = kM;
    }*/
    public double updateViper(double targetHeight, double currentHeight, double systemTime) {
        if(!super.isLaunched()) {
            launch(currentHeight, systemTime);
            return 0.0;
        }

        this.correction = update(targetHeight, currentHeight, systemTime);
        this.correction += currentHeight/km;
        return correction;
    }
    public double updateArmClamped(double targetHeight, double currentPosition, double systemTime) {
        correction = clampOutput(
                updateViper(targetHeight, clampInput(currentPosition), systemTime)
        );
        return correction;
    }
    @Override
    public double getCorrection() {
        return this.correction;
    }
}
