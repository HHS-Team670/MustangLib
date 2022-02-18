package frc.team670.mustanglib.utils.servo;

/**
 * Represents a type of servo.
 * Each type has a Config attached to it, which should be used when setting PWN bounds.
 * 
 * @author AkshatAdsule
 */
public enum ServoType {
    /**
     * Andymark L16 Linear Servo
     */
    ANDYMARK_L16(new ServoPWMConfig(2.0, 1.8, 1.5, 1.2, 1.0));

    public final ServoPWMConfig config;
    private ServoType(ServoPWMConfig config){
        this.config = config;
    }
}

/**
 * Represents constrains for PWM based servos
 * 
 * @author AkshatAdsule
 */
final class ServoPWMConfig {
    protected final double max, deadbandMax, center, deadbandMin, min;

    protected ServoPWMConfig(double max, double deadbandMax, double center, double deadbandMin, double min) {
        this.max = max;
        this.deadbandMax = deadbandMax;
        this.center = center;
        this.deadbandMin = deadbandMin;
        this.min = min;
    }
}