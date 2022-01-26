package frc.team670.mustanglib.commands;

import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.team670.mustanglib.commands.RamseteCommand;
import frc.team670.mustanglib.constants.RobotConstantsBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.drivebase.DriveBase;
import frc.team670.mustanglib.path.Path;

/**
 * Represents a robot action with defined health requirements for every
 * subsystem it uses.
 * 
 * @author lakshbhambhani
 */
public interface MustangCommand {

    /**
     * @return A Map containing the minimum health condition for each subsystem that
     *         this Command requires to be safely used.
     */
    public Map<MustangSubsystemBase, MustangSubsystemBase.HealthState> getHealthRequirements();

    /**
     * 
     * @param path      The trajectory to follow
     * @param driveBase
     * @return A RamseteCommand which will drive the given trajectory
     */
    default RamseteCommand getTrajectoryFollowerCommand(Path path, DriveBase driveBase) {

        RamseteCommand ramseteCommand = new RamseteCommand(
                path.getTrajectory(),
                driveBase::getPose,
                new RamseteController(RobotConstantsBase.kRamseteB, RobotConstantsBase.kRamseteZeta),
                driveBase.getLeftSimpleMotorFeedforward(),
                driveBase.getRightSimpleMotorFeedforward(),
                driveBase.getKDriveKinematics(),
                driveBase::getWheelSpeeds,
                driveBase.getLeftPIDController(),
                driveBase.getRightPIDController(),
                driveBase::tankDriveVoltage,
                driveBase);

        // Reset odometry to the starting pose of the trajectory. We are currently doing
        // this in the command where the trajectory is getting chosen
        // driveBase.resetOdometry(path.getStartingPose());

        // Run path following command, then stop at the end.
        return ramseteCommand;
    }

    /**
     * 
     * @param path      The trajectory to follow
     * @param driveBase
     * @return A RamseteCommand which will drive the given trajectory
     */
    default RamseteCommand getTrajectoryFollowerCommand(Trajectory path, DriveBase driveBase) {

        RamseteCommand ramseteCommand = new RamseteCommand(
                path,
                driveBase::getPose,
                new RamseteController(RobotConstantsBase.kRamseteB, RobotConstantsBase.kRamseteZeta),
                driveBase.getLeftSimpleMotorFeedforward(),
                driveBase.getRightSimpleMotorFeedforward(),
                driveBase.getKDriveKinematics(),
                driveBase::getWheelSpeeds,
                driveBase.getLeftPIDController(),
                driveBase.getRightPIDController(),
                driveBase::tankDriveVoltage,
                driveBase);

        // Reset odometry to the starting pose of the trajectory. We are currently doing
        // this in the command where the trajectory is getting chosen
        // driveBase.resetOdometry(path.getStartingPose());

        // Run path following command, then stop at the end.
        return ramseteCommand;
    }

}