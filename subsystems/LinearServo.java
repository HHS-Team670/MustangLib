package frc.team670.mustanglib.subsystems;

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
     * @param channel the PWM port 
     */
    public LinearServo(int channel) {
        actuator = new Servo(channel);
        actuator.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    }

    /**
     * Extends the servo to its maximum length
     */
    public void extend() {
        actuator.setSpeed(1);
    }

    /**
     * Extends the servo to a specified length
     * @param distance Distance to extends the servo to. Must be between [0, 1]
     */
    public void setExtent(double distance) {
        actuator.set(distance);
    }

    /**
     * Detracts the servo to its minimum length
     */
    public void detract() {
        actuator.setSpeed(0);
    }  
}
