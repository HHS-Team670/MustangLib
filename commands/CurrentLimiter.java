package frc.team670.mustanglib.commands;

import frc.team670.mustanglib.subsystems.MustangSubsystemBase;

public class CurrentLimiter {
    public MustangSubsystemBase subsytem;
    public int limit;
    public int orginalLimit;
    public CurrentLimiter(MustangSubsystemBase subsystem, int limit){
        this.subsytem = subsystem;
        this.limit = limit;
    }
}