package frc.team670.mustanglib.utils.servo;

/**
 * Represents a type of servo.
 * Each type has a {@link PWMConfig} attached to it, which should be used when setting PWM bounds.
 * 
 * @author AkshatAdsule
 */
public enum ServoType {
    /**
     * Andymark L16 Linear Servo
     */
    ANDYMARK_L16(new PWMConfig(2.0, 1.8, 1.5, 1.2, 1.0));

    public final PWMConfig config;
    private ServoType(PWMConfig config){
        this.config = config;
    }
}

/**
 * Represents constraints for PWM based servos
 * 
 * @author AkshatAdsule
 */
final class PWMConfig {
    protected final double max, deadbandMax, center, deadbandMin, min;

    protected PWMConfig(double max, double deadbandMax, double center, double deadbandMin, double min) {
        this.max = max;
        this.deadbandMax = deadbandMax;
        this.center = center;
        this.deadbandMin = deadbandMin;
        this.min = min;
    }
}