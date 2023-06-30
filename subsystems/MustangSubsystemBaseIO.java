package frc.team670.mustanglib.subsystems;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;

public abstract class MustangSubsystemBaseIO {
    @AutoLog
    public static class MustangSubsystemBaseIOInputs {
        public HealthState lastHealthState=HealthState.UNKNOWN;
        public boolean failedLastTime = false;

        public static NetworkTableInstance instance = NetworkTableInstance.getDefault(); // Do static variables work
                                                                                         // with Advantage kit?
        public static NetworkTable table = instance.getTable("/SmartDashboard");

    }

    /** Updates the set of loggable inputs. */
    public void updateInputs(MustangSubsystemBaseIOInputs inputs) {
        inputs.lastHealthState = checkHealth();
    }

    /**
     * Calculates the current state of the subsystem.
     */
    public abstract HealthState checkHealth();
}
