package frc.team670.mustanglib.subsystems.servo;

import edu.wpi.first.wpilibj.Servo;


/**
 * Represents a Linear Servo
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
        ServoConfig config = type.config;
        actuator.setBounds(config.max, config.deadbandMax, config.center, config.deadbandMin, config.min);
    }

    /**
     * Extends the servo to its maximum length
     */
    public void extend() {
        actuator.setSpeed(1);
    }

    /**
     * Retracts the servo to its minimum length
     */
    public void retract() {
        actuator.setSpeed(0);
    }  

    /**
     * Extends the servo to a specified length
     * @param distance Distance to extends the servo to. Must be between [0, 1]
     */
    public void setExtent(double distance) {
        if(Math.abs(distance) > 1) {
            throw new IllegalArgumentException("Distance must be between [0, 1]");
        }
        actuator.setSpeed(distance);
    }
}