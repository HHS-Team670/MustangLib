package frc.team670.mustanglib.subsystems.drivebase;

import java.util.Map;
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
import frc.team670.mustanglib.RobotBase;
import frc.team670.mustanglib.dataCollection.sensors.NavX;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.VisionSubsystemBase;
import frc.team670.mustanglib.swervelib.Mk4iSwerveModuleHelper;
import frc.team670.mustanglib.swervelib.Mk4iSwerveModuleHelper.GearRatio;
import frc.team670.mustanglib.swervelib.SwerveModule;
import frc.team670.mustanglib.swervelib.pathplanner.MustangPPSwerveControllerCommand;
import frc.team670.mustanglib.utils.SwervePoseEstimator;
import frc.team670.robot.constants.RobotConstants;

public abstract class SwerveDrive extends MustangSubsystemBase {

    // private SwerveDriveOdometry odometer;
    private SwervePoseEstimator mPoseEstimator;
    private final NavX mNavx;
    private VisionSubsystemBase mVision;

    MustangPPSwerveControllerCommand mSwerveControllerCommand;

    private final SwerveModule[] mModules;
    private final SwerveDriveKinematics kKinematics;
    private ChassisSpeeds mChassisSpeeds;
    private Rotation2d mGyroOffset = new Rotation2d();
    private Rotation2d mDesiredHeading = null; // for rotation snapping

    private double frontLeftPrevAngle, frontRightPrevAngle, backLeftPrevAngle, backRightPrevAngle;

    private final double MAX_VELOCITY, MAX_VOLTAGE;


    public static record Config(double kDriveBaseTrackWidth, double kDriveBaseWheelBase,
            double kMaxVelocity, double kMaxVoltage, SerialPort.Port kNavXPort,
            GearRatio kSwerveModuleGearRatio, int kFrontLeftModuleDriveMotor,
            int kFrontLeftModuleSteerMotor, int kFrontLeftModuleSteerEncoder,
            double kFrontLeftModuleSteerOffset, int kFrontRightModuleDriveMotor,
            int kFrontRightModuleSteerMotor, int kFrontRightModuleSteerEncoder,
            double kFrontRightModuleSteerOffset, int kBackLeftModuleDriveMotor,
            int kBackLeftModuleSteerMotor, int kBackLeftModuleSteerEncoder,
            double kBackLeftModuleSteerOffset, int kBackRightModuleDriveMotor,
            int kBackRightModuleSteerMotor, int kBackRightModuleSteerEncoder,
            double kBackRightModuleSteerOffset) {
    }

    public SwerveDrive(Config config) {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        MAX_VELOCITY = config.kMaxVelocity;
        MAX_VOLTAGE = config.kMaxVoltage;
        mModules = new SwerveModule[4];

        // front left
        mModules[0] = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4)
                        .withPosition(0, 0),
                config.kSwerveModuleGearRatio, config.kFrontLeftModuleDriveMotor,
                config.kFrontLeftModuleSteerMotor, config.kFrontLeftModuleSteerEncoder,
                config.kFrontLeftModuleSteerOffset);

        // front right
        mModules[1] = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4)
                        .withPosition(2, 0),
                config.kSwerveModuleGearRatio, config.kFrontRightModuleDriveMotor,
                config.kFrontRightModuleSteerMotor, config.kFrontRightModuleSteerEncoder,
                config.kFrontRightModuleSteerOffset);

        // back left
        mModules[2] = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4)
                        .withPosition(4, 0),
                config.kSwerveModuleGearRatio, config.kBackLeftModuleDriveMotor,
                config.kBackLeftModuleSteerMotor, config.kBackLeftModuleSteerEncoder,
                config.kBackLeftModuleSteerOffset);

        // back right
        mModules[3] = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4)
                        .withPosition(6, 0),
                config.kSwerveModuleGearRatio, config.kBackRightModuleDriveMotor,
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
        mChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        // odometer = new SwerveDriveOdometry(getSwerveKinematics(), new Rotation2d(0),
        // getModulePositions());
        mPoseEstimator = new SwervePoseEstimator(this);

        SmartDashboard.putNumber("MAX VELOCITY M/S", MAX_VELOCITY);
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
        mChassisSpeeds = chassisSpeeds;
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
        return mChassisSpeeds;
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

        if (RobotBase.getInstance().isTeleopEnabled()
                && (mSwerveControllerCommand == null || !mSwerveControllerCommand.isScheduled())) {
            SwerveModuleState[] states = kKinematics.toSwerveModuleStates(mChassisSpeeds);
            setModuleStates(states);
        }
    }

    public void initVision(VisionSubsystemBase vision) {
        this.mVision = vision;
    }

    public SwervePoseEstimator getPoseEstimator() {
        return mPoseEstimator;
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

        mModules[0].set(frontLeftSpeed, frontLeftAngle);
        mModules[1].set(frontRightSpeed, frontRightAngle);
        mModules[2].set(backLeftSpeed, backLeftAngle);
        mModules[3].set(backRightSpeed, backRightAngle);

        frontLeftPrevAngle = frontLeftAngle;
        frontRightPrevAngle = frontRightAngle;
        backLeftPrevAngle = backLeftAngle;
        backRightPrevAngle = backRightAngle;

        // for (SwerveModule m : mModules) {
        // SmartDashboard.putString(m.toString(), String.format("velocity: %f\nangle:
        // %f",
        // m.getDriveVelocity(), m.getSteerAngle()));
        // }
    }

    public void realignModules() {
        mModules[2].realign();
        mModules[3].realign();
        mModules[0].realign();
        mModules[1].realign();
    }

    public Pose2d getPose() {
        // return odometer.getPoseMeters();
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
        // odometer.resetPosition(pose.getRotation(), getModulePositions(), pose);
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

    public void setmSwerveControllerCommand(MustangPPSwerveControllerCommand command) {
        this.mSwerveControllerCommand = command;
    }

    public MustangPPSwerveControllerCommand getSwerveControllerCommand() {
        return this.mSwerveControllerCommand;
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
}
