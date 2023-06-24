package frc.team670.mustanglib.subsystems.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.utils.GeometryUtils;

public class CorrectSwerveKinematics extends SwerveDriveKinematics {

    public CorrectSwerveKinematics(Translation2d... wheelsMeters) {
        super(wheelsMeters);
    }

    @Override
    public SwerveModuleState[] toSwerveModuleStates(
        ChassisSpeeds chassisSpeeds, Translation2d centerOfRotationMeters) {

        chassisSpeeds = correctForDynamics(chassisSpeeds);
        return super.toSwerveModuleStates(chassisSpeeds, centerOfRotationMeters);
        
    }

    private ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {

        SmartDashboard.putBoolean("correct for dynamics executed", true);

        final double LOOP_TIME_S = 0.02;
        Pose2d futureRobotPose =
            new Pose2d(
                originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
                originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
                Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
        Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
        ChassisSpeeds updatedSpeeds =
            new ChassisSpeeds(
                twistForPose.dx / LOOP_TIME_S,
                twistForPose.dy / LOOP_TIME_S,
                twistForPose.dtheta / LOOP_TIME_S);
        return updatedSpeeds;
      }
}