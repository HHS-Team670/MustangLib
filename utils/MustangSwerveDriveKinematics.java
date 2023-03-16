package frc.team670.mustanglib.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.team670.mustanglib.constants.RobotConstantsBase;

public class MustangSwerveDriveKinematics extends SwerveDriveKinematics {

    public MustangSwerveDriveKinematics(Translation2d... wheelsMeters) {
        super(wheelsMeters);
    }

    @Override
    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds) {

        Pose2d robot_pose_vel = new Pose2d(chassisSpeeds.vxMetersPerSecond * RobotConstantsBase.LOOP_TIME,
                chassisSpeeds.vyMetersPerSecond * RobotConstantsBase.LOOP_TIME,
                Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond * RobotConstantsBase.LOOP_TIME));
        Twist2d t2d = (new Pose2d()).log(robot_pose_vel);

        ChassisSpeeds updatedChassisSpeeds = new ChassisSpeeds(t2d.dx, t2d.dy, t2d.dtheta);
        return super.toSwerveModuleStates(updatedChassisSpeeds);

    }

}