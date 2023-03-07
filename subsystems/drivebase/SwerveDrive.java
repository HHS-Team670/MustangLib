package frc.team670.mustanglib.subsystems.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.RobotBase;
import frc.team670.mustanglib.constants.SwerveConfig;
import frc.team670.mustanglib.dataCollection.sensors.NavX;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.VisionSubsystemBase;
import frc.team670.mustanglib.swervelib.Mk4iSwerveModuleHelper;
import frc.team670.mustanglib.swervelib.SwerveModule;
import frc.team670.mustanglib.utils.SwervePoseEstimator;
import frc.team670.robot.commands.drivebase.MustangPPSwerveControllerCommand;

public abstract class SwerveDrive extends MustangSubsystemBase {

    // private SwerveDriveOdometry odometer;
    private SwervePoseEstimator poseEstimator;
    private final NavX m_navx;
    private VisionSubsystemBase vision;

    MustangPPSwerveControllerCommand swerveControllerCommand;

    private final SwerveModule[] m_modules;
    private final SwerveDriveKinematics m_kinematics;
    private ChassisSpeeds m_chassisSpeeds;
    private Rotation2d gyroOffset = new Rotation2d();
    private Rotation2d desiredHeading = null; // for rotation snapping

    private double frontLeftPrevAngle, frontRightPrevAngle, backLeftPrevAngle, backRightPrevAngle;

    private final double MAX_VELOCITY, MAX_VOLTAGE;

    public SwerveDrive(SwerveConfig config) {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        MAX_VELOCITY = config.MAX_VELOCITY_METERS_PER_SECOND;
        MAX_VOLTAGE = config.MAX_VOLTAGE;
        m_modules = new SwerveModule[4];

        // front left
        m_modules[0] = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4)
                        .withPosition(0, 0),
                Mk4iSwerveModuleHelper.GearRatio.L1, config.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                config.FRONT_LEFT_MODULE_STEER_MOTOR, config.FRONT_LEFT_MODULE_STEER_ENCODER,
                config.FRONT_LEFT_MODULE_STEER_OFFSET);

        // front right
        m_modules[1] = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4)
                        .withPosition(2, 0),
                Mk4iSwerveModuleHelper.GearRatio.L1, config.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                config.FRONT_RIGHT_MODULE_STEER_MOTOR, config.FRONT_RIGHT_MODULE_STEER_ENCODER,
                config.FRONT_RIGHT_MODULE_STEER_OFFSET);

        // back left
        m_modules[2] = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4)
                        .withPosition(4, 0),
                Mk4iSwerveModuleHelper.GearRatio.L1, config.BACK_LEFT_MODULE_DRIVE_MOTOR,
                config.BACK_LEFT_MODULE_STEER_MOTOR, config.BACK_LEFT_MODULE_STEER_ENCODER,
                config.BACK_LEFT_MODULE_STEER_OFFSET);

        // back right
        m_modules[3] = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4)
                        .withPosition(6, 0),
                Mk4iSwerveModuleHelper.GearRatio.L1, config.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                config.BACK_RIGHT_MODULE_STEER_MOTOR, config.BACK_RIGHT_MODULE_STEER_ENCODER,
                config.BACK_RIGHT_MODULE_STEER_OFFSET);

        m_kinematics = new SwerveDriveKinematics(
                // Front left
                new Translation2d(config.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                        config.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                // Front right
                new Translation2d(config.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                        -config.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                // Back left
                new Translation2d(-config.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                        config.DRIVETRAIN_WHEELBASE_METERS / 2.0),
                // Back right
                new Translation2d(-config.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                        -config.DRIVETRAIN_WHEELBASE_METERS / 2.0));

        m_navx = new NavX(config.NAVX_PORT);
        m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        // odometer = new SwerveDriveOdometry(getSwerveKinematics(), new Rotation2d(0),
        // getModulePositions());
        poseEstimator = new SwervePoseEstimator(this);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        // Logger.consoleLog(chassisSpeeds.vxMetersPerSecond + ", " +
        // chassisSpeeds.vyMetersPerSecond + ", " +
        // chassisSpeeds.omegaRadiansPerSecond);
        // SmartDashboard.putNumber("chassis x velocity",
        // chassisSpeeds.vxMetersPerSecond);
        // SmartDashboard.putNumber("chassis y velocity",
        // chassisSpeeds.vyMetersPerSecond);
        // SmartDashboard.putNumber("chassis omega velocity",
        // chassisSpeeds.omegaRadiansPerSecond);
        m_chassisSpeeds = chassisSpeeds;
    }

    public void stop() {
        drive(new ChassisSpeeds());
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the robot is
     * currently facing to the 'forwards' direction.
     */
    public void zeroGyroscope() {
        gyroOffset = getGyroscopeRotation(false);
    }

    public void setGyroscopeRotation(Rotation2d rot) {
        gyroOffset = rot;
    }

    public SwerveDriveKinematics getSwerveKinematics() {
        return m_kinematics;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return m_chassisSpeeds;
    }

    public Rotation2d getGyroscopeRotation() {
        return getGyroscopeRotation(true);
    }

    public void setDesiredHeading(Rotation2d rot) {
        this.desiredHeading = rot;
    }

    public Rotation2d getDesiredHeading() {
        return this.desiredHeading;
    }

    public Rotation2d getGyroscopeRotation(boolean offset) {
        if (m_navx.isMagnetometerCalibrated()) {

            // We will only get valid fused headings if the magnetometer is calibrated
            if (offset) {
                Rotation2d angle =
                        Rotation2d.fromDegrees(-m_navx.getFusedHeading()).minus(gyroOffset);
                return angle;
            }
            return Rotation2d.fromDegrees(-m_navx.getFusedHeading());
        }

        // We have to invert the angle of the NavX so that rotating the robot
        // counter-clockwise
        // makes the angle increase.
        if (offset) {
            return Rotation2d.fromDegrees(-m_navx.getYawFieldCentric()).minus(gyroOffset);
        }
        return Rotation2d.fromDegrees(-m_navx.getYawFieldCentric());
    }

    @Override
    public void mustangPeriodic() {

        if (gyroOffset == null && !m_navx.isCalibrating()) {
            zeroGyroscope();
        }

        if (vision != null) {
            if (poseEstimator.getVision() == null) {
                vision.initalize();     // at this point, DS is initalized. Okay calling vision init here.
                poseEstimator.initialize(vision);
            } else {
                poseEstimator.update();
            }
        }

        if (RobotBase.getInstance().isTeleopEnabled()
                && (swerveControllerCommand == null || !swerveControllerCommand.isScheduled())) {
            SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
            setModuleStates(states);
        }
    }

    public void initVision(VisionSubsystemBase vision) {
        this.vision = vision;
    }

    public SwervePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    public void setModuleStates(SwerveModuleState[] states) {

        double frontLeftSpeed = states[0].speedMetersPerSecond / MAX_VELOCITY * MAX_VOLTAGE;
        double frontRightSpeed = states[1].speedMetersPerSecond / MAX_VELOCITY * MAX_VOLTAGE;
        double backLeftSpeed = states[2].speedMetersPerSecond / MAX_VELOCITY * MAX_VOLTAGE;
        double backRightSpeed = states[3].speedMetersPerSecond / MAX_VELOCITY * MAX_VOLTAGE;

        double frontLeftAngle = states[0].angle.getRadians();
        double frontRightAngle = states[1].angle.getRadians();
        double backLeftAngle = states[2].angle.getRadians();
        double backRightAngle = states[3].angle.getRadians();

        if (Math.abs(frontLeftSpeed) <= 0.01 && Math.abs(frontRightSpeed) <= 0.01
                && Math.abs(backLeftSpeed) <= 0.01 && Math.abs(backRightSpeed) <= 0.01) {
            frontLeftAngle = frontLeftPrevAngle;
            frontRightAngle = frontRightPrevAngle;
            backLeftAngle = backLeftPrevAngle;
            backRightAngle = backRightPrevAngle;
        }

        m_modules[0].set(frontLeftSpeed, frontLeftAngle);
        m_modules[1].set(frontRightSpeed, frontRightAngle);
        m_modules[2].set(backLeftSpeed, backLeftAngle);
        m_modules[3].set(backRightSpeed, backRightAngle);

        frontLeftPrevAngle = frontLeftAngle;
        frontRightPrevAngle = frontRightAngle;
        backLeftPrevAngle = backLeftAngle;
        backRightPrevAngle = backRightAngle;

        for (SwerveModule m : m_modules) {
            SmartDashboard.putString(m.toString(), String.format("velocity: %f\nangle: %f",
                    m.getDriveVelocity(), m.getSteerAngle()));
        }
    }

    public void realignModules() {
        m_modules[2].realign();
        m_modules[3].realign();
        m_modules[0].realign();
        m_modules[1].realign();
    }

    public Pose2d getPose() {
        // return odometer.getPoseMeters();
        return poseEstimator.getCurrentPose();
    }

    public double getPitch() {
        return m_navx.getPitch();
    }

    public void resetOdometry(Pose2d pose) {
        SmartDashboard.putString("reset", "reset");
        SwerveModulePosition[] zeroedPos = new SwerveModulePosition[4];
        for (int i = 0; i < zeroedPos.length; i++) {
            zeroedPos[i] = new SwerveModulePosition();
        }
        setGyroscopeRotation(Rotation2d.fromDegrees(
                getGyroscopeRotation(false).getDegrees() + pose.getRotation().getDegrees()));
        // odometer.resetPosition(pose.getRotation(), getModulePositions(), pose);
        poseEstimator.setCurrentPose(pose);
    }

    public SwerveModule[] getModules() {
        return m_modules;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition positions[] = new SwerveModulePosition[4];
        for (int i = 0; i < m_modules.length; i++) {
            positions[i] = m_modules[i].getPosition();
        }
        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState states[] = new SwerveModuleState[4];
        for (int i = 0; i < m_modules.length; i++) {
            states[i] = m_modules[i].getState();
        }
        return states;
    }

    public void setSwerveControllerCommand(MustangPPSwerveControllerCommand command) {
        this.swerveControllerCommand = command;
    }

    public MustangPPSwerveControllerCommand getSwerveControllerCommand() {
        return this.swerveControllerCommand;
    }
}
