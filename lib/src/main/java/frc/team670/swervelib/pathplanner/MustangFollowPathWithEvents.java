package frc.team670.mustanglib.swervelib.pathplanner;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
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
    public MustangFollowPathWithEvents(MustangPPSwerveControllerCommand command,PathPlannerPath path,Supplier<Pose2d> poseSupplier) {
        super(command, path,poseSupplier);
    }

    /**
     * Gets the health requirements for this command.
     * @return A map of {@link MustangSubsystemBase} to {@link HealthState} representing the health requirements.
     */
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }



}
