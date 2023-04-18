package frc.team670.mustanglib.subsystems.drivebase;

import java.util.Map;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team670.mustanglib.dataCollection.sensors.NavX;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.VisionSubsystemBase;
import frc.team670.mustanglib.swervelib.Mk4ModuleConfiguration;
import frc.team670.mustanglib.swervelib.Mk4iSwerveModuleHelper;
import frc.team670.mustanglib.swervelib.Mk4iSwerveModuleHelper.GearRatio;
import frc.team670.mustanglib.swervelib.SwerveModule;
import frc.team670.mustanglib.swervelib.pathplanner.MustangPPSwerveControllerCommand;
import frc.team670.mustanglib.utils.SwervePoseEstimator;
import frc.team670.robot.constants.RobotConstants;

public abstract class SwerveDrive extends MustangSubsystemBase {
    private SwervePoseEstimator mPoseEstimator;
    private final NavX mNavx;
    private VisionSubsystemBase mVision;

    private final SwerveModule[] mModules;
    private final SwerveDriveKinematics kKinematics;
    private Rotation2d mGyroOffset = new Rotation2d();
    private Rotation2d mDesiredHeading = null; // for rotation snapping

    private final double kMaxVelocity, kMaxVoltage;

    private final Mk4ModuleConfiguration kModuleConfig = new Mk4ModuleConfiguration();

    public static record Config(double kDriveBaseTrackWidth, double kDriveBaseWheelBase,
            double kMaxVelocity, double kMaxVoltage, double kMaxDriveCurrent,
            double kMaxSteerCurrent, SerialPort.Port kNavXPort, GearRatio kSwerveModuleGearRatio,
            int kFrontLeftModuleDriveMotor, int kFrontLeftModuleSteerMotor,
            int kFrontLeftModuleSteerEncoder, double kFrontLeftModuleSteerOffset,
            int kFrontRightModuleDriveMotor, int kFrontRightModuleSteerMotor,
            int kFrontRightModuleSteerEncoder, double kFrontRightModuleSteerOffset,
            int kBackLeftModuleDriveMotor, int kBackLeftModuleSteerMotor,
            int kBackLeftModuleSteerEncoder, double kBackLeftModuleSteerOffset,
            int kBackRightModuleDriveMotor, int kBackRightModuleSteerMotor,
            int kBackRightModuleSteerEncoder, double kBackRightModuleSteerOffset) {
    }

    public SwerveDrive(Config config) {
        kMaxVelocity = config.kMaxVelocity;
        kMaxVoltage = config.kMaxVoltage;

        kModuleConfig.setDriveCurrentLimit(kMaxVoltage);
        kModuleConfig.setDriveCurrentLimit(config.kMaxDriveCurrent);
        kModuleConfig.setSteerCurrentLimit(config.kMaxSteerCurrent);

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        mModules = new SwerveModule[4];


        // front left
        mModules[0] = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4)
                        .withPosition(0, 0),
                kModuleConfig, config.kSwerveModuleGearRatio, config.kFrontLeftModuleDriveMotor,
                config.kFrontLeftModuleSteerMotor, config.kFrontLeftModuleSteerEncoder,
                config.kFrontLeftModuleSteerOffset);

        // front right
        mModules[1] = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4)
                        .withPosition(2, 0),
                kModuleConfig, config.kSwerveModuleGearRatio, config.kFrontRightModuleDriveMotor,
                config.kFrontRightModuleSteerMotor, config.kFrontRightModuleSteerEncoder,
                config.kFrontRightModuleSteerOffset);

        // back left
        mModules[2] = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4)
                        .withPosition(4, 0),
                kModuleConfig, config.kSwerveModuleGearRatio, config.kBackLeftModuleDriveMotor,
                config.kBackLeftModuleSteerMotor, config.kBackLeftModuleSteerEncoder,
                config.kBackLeftModuleSteerOffset);

        // back right
        mModules[3] = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4)
                        .withPosition(6, 0),
                kModuleConfig, config.kSwerveModuleGearRatio, config.kBackRightModuleDriveMotor,
                config.kBackRightModuleSteerMotor, config.kBackRightModuleSteerEncoder,
                config.kBackRightModuleSteerOffset);

        kKinematics = new SwerveDriveKinematics(
                // Front left
                new Translation2d(config.kDriveBaseTrackWidth / 2.0,
                        config.kDriveBaseWheelBase / 2.0),
                // Front right
                new Translation2d(config.kDriveBaseTrackWidth / 2.0,
                        -config.kDriveBaseWheelBase / 2.0),
                // Back left
                new Translation2d(-config.kDriveBaseTrackWidth / 2.0,
                        config.kDriveBaseWheelBase / 2.0),
                // Back right
                new Translation2d(-config.kDriveBaseTrackWidth / 2.0,
                        -config.kDriveBaseWheelBase / 2.0));

        mNavx = new NavX(config.kNavXPort);
        // mChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        // odometer = new SwerveDriveOdometry(getSwerveKinematics(), new Rotation2d(0),
        // getModulePositions());
        mPoseEstimator = new SwervePoseEstimator(this);

        SmartDashboard.putNumber("MAX VELOCITY M/S", config.kMaxVelocity);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        setModuleStates(kKinematics.toSwerveModuleStates(chassisSpeeds));
    }

    public void stop() {
        drive(new ChassisSpeeds());
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the robot is
     * currently facing to the 'forwards' direction.
     */
    public void zeroGyroscope() {
        mGyroOffset = getGyroscopeRotation(false);
    }

    public void setGyroscopeRotation(Rotation2d rot) {
        mGyroOffset = rot;
    }

    public SwerveDriveKinematics getSwerveKinematics() {
        return kKinematics;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kKinematics.toChassisSpeeds(getModuleStates());
    }

    public Rotation2d getGyroscopeRotation() {
        return getGyroscopeRotation(true);
    }

    public void setmDesiredHeading(Rotation2d rot) {
        this.mDesiredHeading = rot;
    }

    public Rotation2d getDesiredHeading() {
        return this.mDesiredHeading;
    }

    public Rotation2d getGyroscopeRotation(boolean offset) {
        if (mNavx.isMagnetometerCalibrated()) {

            // We will only get valid fused headings if the magnetometer is calibrated
            if (offset) {
                Rotation2d angle =
                        Rotation2d.fromDegrees(-mNavx.getFusedHeading()).minus(mGyroOffset);
                SmartDashboard.putNumber("gyro offset", mGyroOffset.getDegrees());
                return angle;
            }
            return Rotation2d.fromDegrees(-mNavx.getFusedHeading());
        }

        // We have to invert the angle of the NavX so that rotating the robot
        // counter-clockwise
        // makes the angle increase.
        if (offset) {
            return Rotation2d.fromDegrees(-mNavx.getYawFieldCentric()).minus(mGyroOffset);
        }
        return Rotation2d.fromDegrees(-mNavx.getYawFieldCentric());
    }

    @Override
    public void mustangPeriodic() {

        if (mGyroOffset == null && !mNavx.isCalibrating()) {
            zeroGyroscope();
            realignModules();
        }

        if (mVision != null) {
            if (mPoseEstimator.getVision() == null) {
                if (!mVision.isInitialized())
                    mVision.initalize(); // at this point, DS is initalized. Okay calling vision
                                         // init
                                         // here.
                mPoseEstimator.initialize(mVision);
            }
        }
        mPoseEstimator.update();
        SmartDashboard.putNumber("navX heading", getPose().getRotation().getDegrees());
    }

    public void initVision(VisionSubsystemBase vision) {
        this.mVision = vision;
    }

    public SwervePoseEstimator getPoseEstimator() {
        return mPoseEstimator;
    }

    public void setModuleStates(SwerveModuleState[] states) {

        double frontLeftSpeed = states[0].speedMetersPerSecond / kMaxVelocity * kMaxVoltage;
        double frontRightSpeed = states[1].speedMetersPerSecond / kMaxVelocity * kMaxVoltage;
        double backLeftSpeed = states[2].speedMetersPerSecond / kMaxVelocity * kMaxVoltage;
        double backRightSpeed = states[3].speedMetersPerSecond / kMaxVelocity * kMaxVoltage;

        double frontLeftAngle = states[0].angle.getRadians();
        double frontRightAngle = states[1].angle.getRadians();
        double backLeftAngle = states[2].angle.getRadians();
        double backRightAngle = states[3].angle.getRadians();

        // angle check doesn't do anything. Probably contributes to error. Gets the same angle as
        // prevAngle since the only time the angle is set is after retrieving these values.

        // if (Math.abs(frontLeftSpeed) <= 0.01 && Math.abs(frontRightSpeed) <= 0.01
        // && Math.abs(backLeftSpeed) <= 0.01 && Math.abs(backRightSpeed) <= 0.01) {
        // frontLeftAngle = frontLeftPrevAngle;
        // frontRightAngle = frontRightPrevAngle;
        // backLeftAngle = backLeftPrevAngle;
        // backRightAngle = backRightPrevAngle;
        // }

        mModules[0].set(frontLeftSpeed, frontLeftAngle);
        mModules[1].set(frontRightSpeed, frontRightAngle);
        mModules[2].set(backLeftSpeed, backLeftAngle);
        mModules[3].set(backRightSpeed, backRightAngle);

        // frontLeftPrevAngle = frontLeftAngle;
        // frontRightPrevAngle = frontRightAngle;
        // backLeftPrevAngle = backLeftAngle;
        // backRightPrevAngle = backRightAngle;
    }

    public void realignModules() {
        for (SwerveModule m : mModules)
            m.realign();
    }

    public Pose2d getPose() {
        return mPoseEstimator.getCurrentPose();
    }

    public double getPitch() {
        return mNavx.getPitch() - RobotConstants.DriveBase.kPitchOffset;
    }

    public void resetOdometry(Pose2d pose) {
        SwerveModulePosition[] zeroedPos = new SwerveModulePosition[4];
        for (int i = 0; i < zeroedPos.length; i++) {
            zeroedPos[i] = new SwerveModulePosition();
        }
        setGyroscopeRotation(Rotation2d.fromDegrees(
                getGyroscopeRotation(false).getDegrees() + pose.getRotation().getDegrees()));
        mPoseEstimator.setCurrentPose(pose);
    }

    public SwerveModule[] getModules() {
        return mModules;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition positions[] = new SwerveModulePosition[4];
        for (int i = 0; i < mModules.length; i++) {
            positions[i] = mModules[i].getPosition();
        }
        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState states[] = new SwerveModuleState[4];
        for (int i = 0; i < mModules.length; i++) {
            states[i] = mModules[i].getState();
        }
        return states;
    }

    public void park() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        // needs the 0.1 or else it won't even rotate the wheels
        states[0] = new SwerveModuleState(0.1, new Rotation2d(Math.PI / 4)); // front right
        states[1] = new SwerveModuleState(0.1, new Rotation2d(-Math.PI / 4)); // front left
        states[2] = new SwerveModuleState(0.1, new Rotation2d(-Math.PI / 4)); // back left
        states[3] = new SwerveModuleState(0.1, new Rotation2d(Math.PI / 4)); // back right
        setModuleStates(states);
    }

    public SwerveAutoBuilder getAutoBuilderFromEvents(Map<String, Command> eventMap) {
        return new SwerveAutoBuilder(this::getPose, this::resetOdometry, kKinematics,
                RobotConstants.DriveBase.kAutonTranslationPID,
                RobotConstants.DriveBase.kAutonThetaPID, this::setModuleStates, eventMap, true,
                new Subsystem[] {this});
    }

    public MustangPPSwerveControllerCommand getFollowTrajectoryCommand(PathPlannerTrajectory traj) {
        return new MustangPPSwerveControllerCommand(traj, this::getPose, getSwerveKinematics(),
                RobotConstants.DriveBase.xController, RobotConstants.DriveBase.yController,
                RobotConstants.DriveBase.thetaController, this::setModuleStates,
                new Subsystem[] {this});
    }
}
