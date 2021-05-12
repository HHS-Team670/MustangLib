package frc.team670.mustanglib.commands;

import java.util.Map;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.team670.mustanglib.commands.RamseteCommand;
import frc.team670.mustanglib.constants.RobotConstantsBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.drivebase.TankDriveBase;
import frc.team670.paths.Path;

/**
 * Represents a robot action with defined health requirements for every subsystem it uses.
 * 
 * @author lakshbhambhani
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
        // PIDController leftPIDController = new PIDController(RobotConstantsBase.leftKPDriveVel,
        //         RobotConstantsBase.leftKIDriveVel, RobotConstantsBase.leftKDDriveVel);
        // PIDController rightPIDController = new PIDController(RobotConstantsBase.rightKPDriveVel,
        //         RobotConstantsBase.rightKIDriveVel, RobotConstantsBase.rightKDDriveVel);

       

        // RamseteCommand ramseteCommand = new RamseteCommand(path.getTrajectory(), driveBase::getPose,
        //         new RamseteController(RobotConstantsBase.kRamseteB, RobotConstantsBase.kRamseteZeta),
        //         new SimpleMotorFeedforward(RobotConstantsBase.leftKsVolts, RobotConstantsBase.leftKvVoltSecondsPerMeter,
        //                 RobotConstantsBase.leftKaVoltSecondsSquaredPerMeter),
        //         new SimpleMotorFeedforward(RobotConstantsBase.rightKsVolts, RobotConstantsBase.rightKvVoltSecondsPerMeter,
        //                 RobotConstantsBase.rightKaVoltSecondsSquaredPerMeter),
        //         RobotConstantsBase.kDriveKinematics, driveBase::getWheelSpeeds, leftPIDController, rightPIDController,
        //         driveBase::tankDriveVoltage, driveBase);

        //PIDController leftPIDController = driveBase::getLeftPIDController;
        //PIDController rightPIDController =  driveBase::getRightPIDController;

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

}