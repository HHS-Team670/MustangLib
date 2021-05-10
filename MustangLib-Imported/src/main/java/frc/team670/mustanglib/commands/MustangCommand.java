package frc.team670.mustanglib.commands;

import java.util.Map;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.team670.mustanglib.constants.RobotConstants;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.drivebase.TankDriveBase;

/**
 * Represents a robot action with defined health requirements for every subsystem it uses.
 * 
 * @author ctychen, lakshbhambhani
 */
public interface MustangCommand{

    /**
     * @return A Map containing the minimum health condition for each subsystem that this Command requires to be safely used.
     */
    public Map<MustangSubsystemBase, MustangSubsystemBase.HealthState> getHealthRequirements();

       /**
     * 
     * @param path      The trajectory to follow
     * @param driveBase
     * @return A RamseteCommand which will drive the given trajectory
     */
    default RamseteCommand getTrajectoryFollowerCommand(Path path, TankDriveBase driveBase) {
        PIDController leftPIDController = new PIDController(RobotConstants.leftKPDriveVel,
                RobotConstants.leftKIDriveVel, RobotConstants.leftKDDriveVel);
        PIDController rightPIDController = new PIDController(RobotConstants.rightKPDriveVel,
                RobotConstants.rightKIDriveVel, RobotConstants.rightKDDriveVel);

       

        RamseteCommand ramseteCommand = new RamseteCommand(path.getTrajectory(), driveBase::getPose,
                new RamseteController(RobotConstants.kRamseteB, RobotConstants.kRamseteZeta),
                new SimpleMotorFeedforward(RobotConstants.leftKsVolts, RobotConstants.leftKvVoltSecondsPerMeter,
                        RobotConstants.leftKaVoltSecondsSquaredPerMeter),
                new SimpleMotorFeedforward(RobotConstants.rightKsVolts, RobotConstants.rightKvVoltSecondsPerMeter,
                        RobotConstants.rightKaVoltSecondsSquaredPerMeter),
                RobotConstants.kDriveKinematics, driveBase::getWheelSpeeds, leftPIDController, rightPIDController,
                driveBase::tankDriveVoltage, driveBase);

        // Reset odometry to the starting pose of the trajectory. We are currently doing
        // this in the command where the trajectory is getting chosen
        // driveBase.resetOdometry(path.getStartingPose());

        // Run path following command, then stop at the end.
        return ramseteCommand;
    }

}