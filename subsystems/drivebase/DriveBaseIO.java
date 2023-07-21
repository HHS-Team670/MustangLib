package frc.team670.mustanglib.subsystems.drivebase;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.MustangSubsystemBaseIO;

public abstract class DriveBaseIO extends MustangSubsystemBaseIO {
    
    @AutoLog
    public static class DriveBaseIOInputs{
        public double gyroYawRad;
        public double gyroPitchRad;
    }


    public abstract void resetHeading();






}
