package frc.team670.mustanglib.commands;

import frc.team670.mustanglib.subsystems.MustangSubsystemBase;

/**
 * stores the subsytem and speed limit for subsytem in RPM used for DefenseModeCommand
 */
public class CurrentLimiter {
    public MustangSubsystemBase subsytem;
    public int limit;

    public CurrentLimiter(MustangSubsystemBase subsystem, int limit){
        this.subsytem = subsystem;
        this.limit = limit;
    }
}