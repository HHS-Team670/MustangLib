package frc.team670.mustanglib.utils.servo;

public enum ServoType {
    /**
     * Andymark L16 Linear Servo
     */
    ANDYMARK_L16(new PWMConfig(2, 2, 1, 1, 1)); //not correct please change

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
    protected final int max, deadbandMax, center, deadbandMin, min;

    protected PWMConfig(int max, int deadbandMax, int center, int deadbandMin, int min) {
        this.max = max;
        this.deadbandMax = deadbandMax;
        this.center = center;
        this.deadbandMin = deadbandMin;
        this.min = min;
    }
}
