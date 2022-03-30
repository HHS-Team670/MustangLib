package frc.team670.mustanglib.utils.servo;

import edu.wpi.first.wpilibj.Servo;

/**
 * Interfaces with a Linear Servo through PWM
 * 
 * @author AkshatAdsule
 */
public class LinearServo {
    
    private final Servo actuator;

    /**
     * Creates a new LinearServo
     * @param channel The PWM port 
     * @param type The type of servo
     */
    public LinearServo(int channel, ServoType type) {
        actuator = new Servo(channel);
        PWMConfig config = type.config;
        actuator.setBounds(config.max, config.deadbandMax, config.center, config.deadbandMin, config.min);
    }

    /**
     * Extends the servo to its maximum length
     */
    public void extend() {
        actuator.set(1);
    }

    /**
     * Retracts the servo to its minimum length
     */
    public void retract() {
        actuator.set(0);
    }  

    /**
     * Extends the servo to a specified length
     * @param distance Distance to extend the servo to. Must be between [0, 1]
     */
    public void setExtent(double distance) {
        if(distance > 1 || distance < 0) {
            throw new IllegalArgumentException("Distance must be between [0, 1]");
        }
        actuator.set(distance);
    }
}