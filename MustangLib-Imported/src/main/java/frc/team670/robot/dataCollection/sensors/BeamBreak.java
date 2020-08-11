package frc.team670.robot.dataCollection.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Used to get inputs from an IR Sensor
 * 
 * @author ctychen
 */
public class BeamBreak {

    private static DigitalInput dio;

    /**
     * Constructs a BeamBreak sensor
     * 
     * @param dioPort The port on which the sensor is
     */
    public BeamBreak(int dioPort) {
        dio = new DigitalInput(dioPort);
    }

    /**
     * @return boolean isTriggered True if the sensor is triggered
     */
    public boolean isTriggered() {
        return (!dio.get());
    }

    /**
     * Writes the state of this beam break sensor to SmartDashboard
     */
    public void sendBeamBreakDataToDashboard() {
        SmartDashboard.putBoolean("Beam Break on" + dio + ":", isTriggered());
    }

}