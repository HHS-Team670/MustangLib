package frc.team670.mustanglib.subsystems.drivebase;

import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;

public abstract class SwerveDrive extends MustangSubsystemBase {

    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;

    private ChassisSpeeds m_chassisSpeeds;

    public SwerveDrive(SwerveModule frontLeftModule, SwerveModule frontRightModule, SwerveModule backLeftModule, SwerveModule backRightModule) {
        m_frontLeftModule = frontLeftModule;
        m_frontRightModule = frontRightModule;
        m_backLeftModule = backLeftModule;
        m_backRightModule = backRightModule;
        m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
    }

    public abstract void resetOdometry(Pose2d pose);
    
    public abstract Pose2d getPose();
    
    public abstract SwerveDriveKinematics getSwerveDriveKinematics(); 

    public HealthState checkHealth(boolean isFrontLeftError, boolean isFrontRightError, boolean isBackLeftError, boolean isBackRightError) {
        HealthState state = HealthState.GREEN;
    
        if (isFrontLeftError || isFrontRightError || isBackLeftError || isBackRightError) {
            state = HealthState.RED;
        }
        return state;
    }

}
