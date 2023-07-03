package frc.team670.mustanglib.dataCollection.sensors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Used to get inputs from an beam break sensor
 * 
 * @author ctychen
 */
public class BeamBreak {

    private DigitalInput dio;
    private int port;

    /**
     * Constructs a BeamBreak sensor
     * 
     * @param dioPort The port on which the sensor is
     */
    public BeamBreak(int dioPort) {
        dio = new DigitalInput(dioPort);
        this.port = dioPort;
    }

    /**
     * @return boolean isTriggered True if the sensor is triggered 
     * (the beam is broken meaning something is between the transmitter and the reciever)
     */
    public boolean isTriggered() {
        return (!dio.get());
    }

    /**
     * Writes the state of this beam break sensor to SmartDashboard
     */
    public void sendBeamBreakDataToDashboard() {
        SmartDashboard.putBoolean("Beam Break on " + port + ":", isTriggered());
    }

}