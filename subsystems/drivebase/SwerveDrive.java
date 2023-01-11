package frc.team670.mustanglib.subsystems.drivebase;

import frc.team670.mustanglib.swervelib.Mk4iSwerveModuleHelper;
import frc.team670.mustanglib.swervelib.SwerveModule;
import frc.team670.mustanglib.utils.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
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
    private SwerveDriveOdometry odometer;

    public SwerveDrive(SwerveConfig config) {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        MAX_VELOCITY = config.MAX_VELOCITY_METERS_PER_SECOND;
        MAX_VOLTAGE = config.MAX_VOLTAGE;

        m_frontLeftModule = Mk4iSwerveModuleHelper.createNeo(
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
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
                    .withPosition(4, 0),
            Mk4iSwerveModuleHelper.GearRatio.L1,
            config.BACK_LEFT_MODULE_DRIVE_MOTOR,
            config.BACK_LEFT_MODULE_STEER_MOTOR,
            config.BACK_LEFT_MODULE_STEER_ENCODER,
            config.BACK_LEFT_MODULE_STEER_OFFSET
        );

        m_backRightModule = Mk4iSwerveModuleHelper.createNeo(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
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
        odometer = new SwerveDriveOdometry(getSwerveKinematics(), new Rotation2d(0));

    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        //Logger.consoleLog(chassisSpeeds.vxMetersPerSecond + ", " + chassisSpeeds.vyMetersPerSecond + ", " + chassisSpeeds.omegaRadiansPerSecond);
        SmartDashboard.putNumber("chassis x velocity", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("chassis y velocity", chassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("chassis omega velocity", chassisSpeeds.omegaRadiansPerSecond);
        m_chassisSpeeds = chassisSpeeds;
    }
    
    /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
    public void zeroGyroscope() {
        gyroOffset = getGyroscopeRotation(false);
    }

    public SwerveDriveKinematics getSwerveKinematics() {
        return m_kinematics;
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
        //Logger.consoleLog(states[0].angle.getDegrees() + ", " + states[1].angle.getDegrees() + ", " + states[2].angle.getDegrees() + ", " + states[3].angle.getDegrees());
        
        if (gyroOffset == null && !m_navx.isCalibrating()) {
            zeroGyroscope();
        }
        
        if(Robot.currentInstance.isTeleopEnabled()) {
            SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
            setModuleStates(states);
        }
        SmartDashboard.putNumber("x: ", odometer.getPoseMeters().getX());
        SmartDashboard.putNumber("y: ", odometer.getPoseMeters().getY());
        SmartDashboard.putNumber("rotation: ", odometer.getPoseMeters().getRotation().getDegrees());
    }

    private int toThousandths(double d) {
        return (int) (d * 1000);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY);

        if(gyroOffset != null) {
            Logger.consoleLog(toThousandths(states[0].speedMetersPerSecond) + ", " + toThousandths(states[1].speedMetersPerSecond) + ", " + toThousandths(states[2].speedMetersPerSecond) + ", " + toThousandths(states[3].speedMetersPerSecond));
            odometer.update(getGyroscopeRotation(), states[0], states[1], states[2], states[3]);
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

    public void realignModules(){
        m_backLeftModule.realign();
        m_backRightModule.realign();
        m_frontLeftModule.realign();
        m_frontRightModule.realign();
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
  }
  
    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(pose, getGyroscopeRotation());
    }

}
