package frc.team670.mustanglib.subsystems.servo;

/**
 * Represents a type of servo.
 * Each type has a Config attached to it, which should be used when setting PWN bounds.
 */
public enum ServoType {
    /**
     * Andymark L16 Linear Servo
     */
    ANDYMARK_L16(new ServoConfig(2.0, 1.8, 1.5, 1.2, 1.0));

    public final ServoConfig config;
    private ServoType(ServoConfig config){
        this.config = config;
    }
}