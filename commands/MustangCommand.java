package frc.team670.mustanglib.commands;

import java.util.Map;

import frc.team670.mustanglib.subsystems.MustangSubsystemBase;

/**
 * Represents a robot action with defined health requirements for every
 * subsystem it uses.
 * 
 * @author lakshbhambhani, ethan c, armaan, aditi
 */
public interface MustangCommand{

    /**
     * @return A Map containing the minimum health condition for each subsystem that
     *         this Command requires to be safely used.
     */
    public Map<MustangSubsystemBase, MustangSubsystemBase.HealthState> getHealthRequirements();

    /**
     * logs data revelant to debugging this command
     * For use when execute or other parts of command have logical errors
     */
    public default void  debugCommand(){}
    /**
     * schedules this command
     */
    default void schedule() {
        MustangScheduler.getInstance().schedule(this);
    }

    
}