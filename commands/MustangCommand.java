package frc.team670.mustanglib.commands;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;

/**
 * Represents a robot action with defined health requirements for every
 * subsystem it uses.
 * 
 * @author lakshbhambhani, ethan c, armaan, aditi
 */
public interface MustangCommand extends Command {

    /**
     * @return A Map containing the minimum health condition for each subsystem that
     *         this Command requires to be safely used.
     */
    public Map<MustangSubsystemBase, MustangSubsystemBase.HealthState> getHealthRequirements();

    /**
     * logs data revelant to debugging this command
     * For use when execute or other parts of command have logical errors
     */
    public void debugCommand();
    /**
     * schedules this command
     */
    @Override
    default void schedule() {
        MustangScheduler.getInstance().schedule(this);
    }

    
}