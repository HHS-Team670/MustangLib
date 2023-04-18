package frc.team670.mustanglib.commands;

import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team670.mustanglib.commands.RamseteCommand;
import frc.team670.mustanglib.constants.RobotConstantsBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.drivebase.DriveBase;
import frc.team670.mustanglib.path.Path;

/**
 * Represents a robot action with defined health requirements for every
 * subsystem it uses.
 * 
 * @author lakshbhambhani, ethan c
 */
public interface MustangCommand extends Command {

    /**
     * @return A Map containing the minimum health condition for each subsystem that
     *         this Command requires to be safely used.
     */
    public Map<MustangSubsystemBase, MustangSubsystemBase.HealthState> getHealthRequirements();

    /**
     * scheuldes this command
     */
    @Override
    default void schedule() {
        MustangScheduler.getInstance().schedule(this);
    }
}