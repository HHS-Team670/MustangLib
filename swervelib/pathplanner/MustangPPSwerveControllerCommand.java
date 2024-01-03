package frc.team670.mustanglib.swervelib.pathplanner;

import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.commands.FollowPathHolonomic;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;




/**
 * A custom implementation of the PPSwerveControllerCommand that implements the MustangCommand interface.
 * This class is used to control a swerve drive using a PathPlannerTrajectory.
 */
public class MustangPPSwerveControllerCommand extends FollowPathHolonomic implements MustangCommand {

    
    public MustangPPSwerveControllerCommand(PathPlannerPath path, Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> currentRobotRelativeSpeeds,Consumer<ChassisSpeeds> output, HolonomicPathFollowerConfig config,
            Subsystem[] requirements) {

        super(path, poseSupplier, currentRobotRelativeSpeeds, output, config, requirements);
    }

    /**
     * Gets the health requirements for the subsystems used by this command.
     * @return A map of MustangSubsystemBase objects to their required HealthState.
     */
    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }


    
}