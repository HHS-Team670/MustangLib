package frc.team670.mustanglib.swervelib.pathplanner;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;


/**
 * A command that follows a path with events and implements the {@link MustangCommand} interface.
 */
public class MustangFollowPathWithEvents extends FollowPathWithEvents implements MustangCommand {
    
    /**
     * Constructs a new {@link MustangFollowPathWithEvents} command.
     * @param command The {@link MustangPPSwerveControllerCommand} to execute.
     * @param eventMarkers The list of {@link EventMarker}s to trigger events.
     * @param eventMap The map of event names to {@link Command}s to execute.
     */
    public MustangFollowPathWithEvents(MustangPPSwerveControllerCommand command, List<EventMarker> eventMarkers, HashMap<String, Command> eventMap) {
        super(command, eventMarkers, eventMap);
    }

    /**
     * Gets the health requirements for this command.
     * @return A map of {@link MustangSubsystemBase} to {@link HealthState} representing the health requirements.
     */
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }



}
