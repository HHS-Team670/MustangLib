package frc.team670.mustanglib.dataCollection.sensors;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Used to get inputs from an IR Sensor
 * 
 * @author lakshbhambhani
 */
public class IRSensor {

    private static DigitalInput dio;
    private final String IRSENSOR_IS_TRIGGERED;

    /**
     * Used to declare an IR Sensor on a specific port
     * 
     * @param dioPort The port on which the sensor is
     */
    public IRSensor(int dioPort) {
        dio = new DigitalInput(dioPort);
        IRSENSOR_IS_TRIGGERED = "IRSensor/" + dio + "/Triggered";
    }

    /**
     * Used to check if the IR Sensor is triggered or not
     * 
     * @return boolean isTriggered True if the ir sensor is triggered
     */
    public boolean isTriggered() {
        return (!dio.get());
    }

    /**
     * Write the state of this IR sensor to Advantage Scope
     */
    public void logIRData() {
        Logger.recordOutput(IRSENSOR_IS_TRIGGERED, isTriggered());
    }

}