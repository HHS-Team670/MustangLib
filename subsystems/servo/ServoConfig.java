package frc.team670.mustanglib.subsystems.servo;


/**
 * Represents constrains for PWM based servos
 * 
 * @author AkshatAdsule
 */
final public class ServoConfig {
    public final double max, deadbandMax, center, deadbandMin, min;

    public ServoConfig(double max, double deadbandMax, double center, double deadbandMin, double min) {
        this.max = max;
        this.deadbandMax = deadbandMax;
        this.center = center;
        this.deadbandMin = deadbandMin;
        this.min = min;
    }
}
