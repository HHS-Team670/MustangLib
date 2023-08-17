package frc.team670.mustanglib.subsystems;

import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;

public abstract class MustangSubsystemBaseIO {
    private static NetworkTableInstance instance = NetworkTableInstance.getDefault(); // Do static variables work                                                                                     // with Advantage kit?
    private static NetworkTable table = instance.getTable("/SmartDashboard");
    private  HealthState lastHealthState=HealthState.UNKNOWN;
    protected boolean debugOutputs = true;
    

    /** Updates the set of loggable inputs. */
    public abstract void updateInputs(LoggableInputs inputs);


   
    /**
     * Calculates the current state of the subsystem.
     */
    protected abstract HealthState checkHealth();

        /**
     * 
     * @param check Whether or not the subsystem's health should be (re)calculated.
     *              If false, this method simply returns the last recorded health
     *              state. If true, the method will re-evaluate the subsystem's
     *              current health. Note that before the first time the subsystem is
     *              "used", by default its state is UNKNOWN, and thus its health
     *              will be calculated at this time.
     * @return The latest known state of this subsystem: GREEN, YELLOW, or RED.
     */
    public HealthState getHealth(boolean check){
        if(check){
            lastHealthState=checkHealth();

        }
        return lastHealthState;
    }

    public final boolean shouldDebugOutputs() {
        return debugOutputs;
    }

    public void pushHealthToDashboard(String name) {
        NetworkTableEntry subsystem = table.getEntry(name);
        subsystem.setString(lastHealthState.toString());
        //Optional if health is red or yellow rumble driver controller
       
    }
    public abstract void debugOutputs();

}
