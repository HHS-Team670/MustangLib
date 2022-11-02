package frc.team670.mustanglib.subsystems.drivebase;

import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.team670.mustanglib.constants.SwerveConfig;
import frc.team670.mustanglib.dataCollection.sensors.NavX;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;

public abstract class SwerveDrive extends MustangSubsystemBase {

    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;

    private final SwerveDriveKinematics m_kinematics;

    private final NavX m_navx;

    private ChassisSpeeds m_chassisSpeeds;

    private Rotation2d gyroOffset;
    private double frontLeftPrevAngle, frontRightPrevAngle, backLeftPrevAngle, backRightPrevAngle;
    private double MAX_VELOCITY, MAX_VOLTAGE;

    public SwerveDrive(SwerveConfig config) {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        MAX_VELOCITY = config.MAX_VELOCITY_METERS_PER_SECOND;
        MAX_VOLTAGE = config.MAX_VOLTAGE;

        m_frontLeftModule = Mk4iSwerveModuleHelper.createNeo(
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk4iSwerveModuleHelper.GearRatio.L1,
            config.FRONT_LEFT_MODULE_DRIVE_MOTOR,
            config.FRONT_LEFT_MODULE_STEER_MOTOR,
            config.FRONT_LEFT_MODULE_STEER_ENCODER,
            config.FRONT_LEFT_MODULE_STEER_OFFSET
        );

        m_frontRightModule = Mk4iSwerveModuleHelper.createNeo(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk4iSwerveModuleHelper.GearRatio.L1,
            config.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            config.FRONT_RIGHT_MODULE_STEER_MOTOR,
            config.FRONT_RIGHT_MODULE_STEER_ENCODER,
            config.FRONT_RIGHT_MODULE_STEER_OFFSET
        );

        m_backLeftModule = Mk4iSwerveModuleHelper.createNeo(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk4iSwerveModuleHelper.GearRatio.L1,
            config.BACK_LEFT_MODULE_DRIVE_MOTOR,
            config.BACK_LEFT_MODULE_STEER_MOTOR,
            config.BACK_LEFT_MODULE_STEER_ENCODER,
            config.BACK_LEFT_MODULE_STEER_OFFSET
        );

        m_backRightModule = Mk4iSwerveModuleHelper.createNeo(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk4iSwerveModuleHelper.GearRatio.L1,
            config.BACK_RIGHT_MODULE_DRIVE_MOTOR,
            config.BACK_RIGHT_MODULE_STEER_MOTOR,
            config.BACK_RIGHT_MODULE_STEER_ENCODER,
            config.BACK_RIGHT_MODULE_STEER_OFFSET
        );

        m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(config.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, config.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(config.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -config.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-config.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, config.DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-config.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -config.DRIVETRAIN_WHEELBASE_METERS / 2.0)
        );

        m_navx = new NavX(config.NAVX_PORT);
        m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
    }

    /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
    public void zeroGyroscope() {
        gyroOffset = getGyroscopeRotation(false);
    }

    public Rotation2d getGyroscopeRotation() {
        return getGyroscopeRotation(true);
    }

    public Rotation2d getGyroscopeRotation(boolean offset) {
        if (m_navx.isMagnetometerCalibrated()) {
            // We will only get valid fused headings if the magnetometer is calibrated
            if (offset) {
                Rotation2d angle = Rotation2d.fromDegrees(-m_navx.getFusedHeading()).minus(gyroOffset);
                return angle;  
            }
            return Rotation2d.fromDegrees(-m_navx.getFusedHeading());
        }

        // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
        if (offset) {      
            return Rotation2d.fromDegrees(m_navx.getYawFieldCentric()-360).minus(gyroOffset);
        }    
        return Rotation2d.fromDegrees(m_navx.getYawFieldCentric()-360);
    }   

    @Override
    public void mustangPeriodic() {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY);

        if (gyroOffset == null && !m_navx.isCalibrating()) {
            zeroGyroscope();
        }

        double frontLeftSpeed = states[0].speedMetersPerSecond / MAX_VELOCITY * MAX_VOLTAGE;
        double frontRightSpeed = states[1].speedMetersPerSecond / MAX_VELOCITY * MAX_VOLTAGE;
        double backLeftSpeed = states[2].speedMetersPerSecond / MAX_VELOCITY * MAX_VOLTAGE;
        double backRightSpeed =  states[3].speedMetersPerSecond / MAX_VELOCITY * MAX_VOLTAGE;

        double frontLeftAngle = states[0].angle.getRadians();
        double frontRightAngle = states[1].angle.getRadians();
        double backLeftAngle = states[2].angle.getRadians();
        double backRightAngle = states[3].angle.getRadians();

        if (Math.abs(frontLeftSpeed) <= 0.01 && Math.abs(frontRightSpeed) <= 0.01 && Math.abs(backLeftSpeed) <= 0.01 && Math.abs(backRightSpeed) <= 0.01) {
                frontLeftAngle = frontLeftPrevAngle;
                frontRightAngle = frontRightPrevAngle;
                backLeftAngle = backLeftPrevAngle;
                backRightAngle = backRightPrevAngle;
        } 

        m_frontLeftModule.set(frontLeftSpeed, frontLeftAngle);
        m_frontRightModule.set(frontRightSpeed, frontRightAngle);
        m_backLeftModule.set(backLeftSpeed, backLeftAngle);
        m_backRightModule.set(backRightSpeed, backRightAngle);

        frontLeftPrevAngle = frontLeftAngle;
        frontRightPrevAngle = frontRightAngle;
        backLeftPrevAngle = backLeftAngle;
        backRightPrevAngle = backRightAngle;
    }

}
