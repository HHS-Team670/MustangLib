package frc.team670.mustanglib.subsystems.drivebase;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.auto.AutoBuilder;
import org.littletonrobotics.junction.Logger;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.team670.mustanglib.subsystems.VisionSubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team670.mustanglib.RobotConstantsBase;
import frc.team670.mustanglib.dataCollection.sensors.NavX;
import frc.team670.mustanglib.swervelib.Mk4ModuleConfiguration;
import frc.team670.mustanglib.swervelib.Mk4iSwerveModuleHelper;
import frc.team670.mustanglib.swervelib.Mk4iSwerveModuleHelper.GearRatio;
import frc.team670.mustanglib.swervelib.SwerveModule;
import frc.team670.mustanglib.swervelib.pathplanner.MustangPPSwerveControllerCommand;
import frc.team670.mustanglib.swervelib.redux.AbsoluteEncoderType;
import frc.team670.mustanglib.utils.SwervePoseEstimatorBase;

/**
 * Swerve Drive subsystem with pose estimation.
 * 
 * @author Tarini, Edward, Justin, Ethan C, Armaan, Aditi
 */
public abstract class SwerveDrive extends DriveBase {
    protected SwervePoseEstimatorBase mPoseEstimator;
    private final NavX mNavx;
    private VisionSubsystemBase mVision;
    public SwerveDriveKinematics kinematics;

    private final SwerveModule[] mModules;
    private final SwerveDriveKinematics kKinematics;
    private Rotation2d mGyroOffset = new Rotation2d();
    private Rotation2d mDesiredHeading = null; // for rotation snapping
    private final String DRIVEBASE_MAX_VELOCITY, DRIVEBASE_OFFSET, DRIVEBASE_HEADING_DEGREE, DRIVEBASE_PITCH, DRIVEBASE_ROLL;
    private final double kMaxVelocity, kMaxVoltage;
    private Config kConfig;
    private final Mk4ModuleConfiguration kModuleConfigFrontLeft = new Mk4ModuleConfiguration();
    private final Mk4ModuleConfiguration kModuleConfigFrontRight = new Mk4ModuleConfiguration();
    private final Mk4ModuleConfiguration kModuleConfigBackLeft = new Mk4ModuleConfiguration();
    private final Mk4ModuleConfiguration kModuleConfigBackRight = new Mk4ModuleConfiguration();
    private final double kPitchOffset;
    private final double kRollOffset;

   
    public static record Config(double kDriveBaseTrackWidth, double kDriveBaseWheelBase,
            double kMaxVelocity,double kMaxAngularVelocity, double kMaxVoltage, double kMaxDriveCurrent,
            double kMaxSteerCurrent, SerialPort.Port kNavXPort, GearRatio kSwerveModuleGearRatio,
            int kFrontLeftModuleDriveMotor, int kFrontLeftModuleSteerMotor,
            int kFrontLeftModuleSteerEncoder, double kFrontLeftModuleSteerOffset, AbsoluteEncoderType kFrontLeftModuleEncoderType,
            
            int kFrontRightModuleDriveMotor, int kFrontRightModuleSteerMotor,
            int kFrontRightModuleSteerEncoder, double kFrontRightModuleSteerOffset, AbsoluteEncoderType kFrontRightModuleEncoderType,
            
            int kBackLeftModuleDriveMotor, int kBackLeftModuleSteerMotor,
            int kBackLeftModuleSteerEncoder, double kBackLeftModuleSteerOffset, AbsoluteEncoderType kBackLeftModuleEncoderType,
            
            int kBackRightModuleDriveMotor, int kBackRightModuleSteerMotor,
            int kBackRightModuleSteerEncoder, double kBackRightModuleSteerOffset, AbsoluteEncoderType kBackRightModuleEncoderType
            ) {
    }
    public SwerveDrive(Config config) {
        this.kConfig=config;
        
        kMaxVelocity = config.kMaxVelocity;
        kMaxVoltage = config.kMaxVoltage;

        kModuleConfigFrontLeft.setNominalVoltage(kMaxVoltage);
        kModuleConfigFrontLeft.setDriveCurrentLimit(config.kMaxDriveCurrent);
        kModuleConfigFrontLeft.setSteerCurrentLimit(config.kMaxSteerCurrent);
        kModuleConfigFrontLeft.setSteerEncoderType(config.kFrontLeftModuleEncoderType);


        kModuleConfigFrontRight.setNominalVoltage(kMaxVoltage);
        kModuleConfigFrontRight.setDriveCurrentLimit(config.kMaxDriveCurrent);
        kModuleConfigFrontRight.setSteerCurrentLimit(config.kMaxSteerCurrent);
        kModuleConfigFrontRight.setSteerEncoderType(config.kFrontRightModuleEncoderType);


        kModuleConfigBackLeft.setNominalVoltage(kMaxVoltage);
        kModuleConfigBackLeft.setDriveCurrentLimit(config.kMaxDriveCurrent);
        kModuleConfigBackLeft.setSteerCurrentLimit(config.kMaxSteerCurrent);
        kModuleConfigBackLeft.setSteerEncoderType(config.kBackLeftModuleEncoderType);


        kModuleConfigBackRight.setNominalVoltage(kMaxVoltage);
        kModuleConfigBackRight.setDriveCurrentLimit(config.kMaxDriveCurrent);
        kModuleConfigBackRight.setSteerCurrentLimit(config.kMaxSteerCurrent);
        kModuleConfigBackRight.setSteerEncoderType(config.kBackRightModuleEncoderType);

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        mModules = new SwerveModule[4];

       // front left
       
       mModules[0] = Mk4iSwerveModuleHelper.createNeo(
        tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4)
                .withPosition(0, 0),
        kModuleConfigFrontLeft, config.kSwerveModuleGearRatio, config.kFrontLeftModuleDriveMotor,
        config.kFrontLeftModuleSteerMotor, config.kFrontLeftModuleSteerEncoder,
        config.kFrontLeftModuleSteerOffset);

        // front right
        mModules[1] = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4)
                        .withPosition(2, 0),
                kModuleConfigFrontRight, config.kSwerveModuleGearRatio, config.kFrontRightModuleDriveMotor,
                config.kFrontRightModuleSteerMotor, config.kFrontRightModuleSteerEncoder,
                config.kFrontRightModuleSteerOffset);

        // back left
        mModules[2] = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4)
                        .withPosition(4, 0),
                kModuleConfigBackLeft, config.kSwerveModuleGearRatio, config.kBackLeftModuleDriveMotor,
                config.kBackLeftModuleSteerMotor, config.kBackLeftModuleSteerEncoder,
                config.kBackLeftModuleSteerOffset);

        // back right
        mModules[3] = Mk4iSwerveModuleHelper.createNeo(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4)
                    .withPosition(6, 0),
            kModuleConfigBackRight, config.kSwerveModuleGearRatio, config.kBackRightModuleDriveMotor,
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
        // mPoseEstimator = new SwervePoseEstimatorBase(this);
        initPoseEstimator();
        kPitchOffset = mNavx.getPitch();
        kRollOffset = mNavx.getRoll();

        DRIVEBASE_MAX_VELOCITY = getName()+"/MaxVelocityMps";
        DRIVEBASE_OFFSET = getName()+"/GyroOffset";
        DRIVEBASE_HEADING_DEGREE = getName()+"/NavXHeadingDeg";
        DRIVEBASE_PITCH = getName()+"/pitch";
        DRIVEBASE_ROLL = getName()+"/roll";

        Logger.recordOutput(DRIVEBASE_MAX_VELOCITY, config.kMaxVelocity);
        
    }
    protected abstract void initPoseEstimator();

    public void drive(ChassisSpeeds chassisSpeeds) {
        setModuleStates(kKinematics.toSwerveModuleStates(chassisSpeeds));
    }

    public void stop() {
        drive(new ChassisSpeeds());
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the
     * robot is
     * currently facing to the 'forwards' direction.
     */
    public void zeroHeading() {
        mGyroOffset = getGyroscopeRotation(false);
    }
    public GearRatio getGearRatio(){
        return kConfig.kSwerveModuleGearRatio();
    }
    public double getMaxVelocityMetersPerSecond(){
        return kMaxVelocity;
    }
    
    public double getMaxAngularVelocityMetersPerSecond(){
        return kConfig.kMaxAngularVelocity;
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
    // public double getMaxVelocityMetersPerSecond(){
    //     return  5676.0 / 60.0
    //     * kModuleConfig.getDriveReduction() * kModuleConfig.getWheelDiameter() * Math.PI;
    // }

    public Rotation2d getGyroscopeRotation(boolean offset) {
        if (mNavx.isMagnetometerCalibrated()) {
            // We will only get valid fused headings if the magnetometer is calibrated
            if (offset) {
                Rotation2d angle = Rotation2d.fromDegrees(-mNavx.getFusedHeading()).minus(mGyroOffset);
                Logger.recordOutput(DRIVEBASE_OFFSET, mGyroOffset.getDegrees());
             
                return angle;
            } else {
                return Rotation2d.fromDegrees(-mNavx.getFusedHeading());
            }
        }

        // We have to invert the angle of the NavX so that rotating the robot
        // counter-clockwise makes the angle increase.
        return offset ? Rotation2d.fromDegrees(-mNavx.getYawFieldCentric()).minus(mGyroOffset)
                : Rotation2d.fromDegrees(-mNavx.getYawFieldCentric());
    }

    @Override
    public void mustangPeriodic() {

        if (mGyroOffset == null && !mNavx.isCalibrating()) {
            zeroHeading();
            realignModules();
        }

        if (mVision != null) {
            if (mPoseEstimator.getVision() == null) {
                mPoseEstimator.initVision(mVision);
            }
        }
        mPoseEstimator.update();
        Logger.recordOutput(DRIVEBASE_HEADING_DEGREE, getPose().getRotation().getDegrees());
        Logger.recordOutput(DRIVEBASE_PITCH, getPitch());
        Logger.recordOutput(DRIVEBASE_ROLL, getRoll());


        
    }

    public void initVision(VisionSubsystemBase vision) {
        this.mVision = vision;
    }

    public SwervePoseEstimatorBase getPoseEstimator() {
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

        // angle check doesn't do anything. Probably contributes to error. Gets the same
        // angle as prevAngle since the only time the angle is set is after retrieving
        // these
        // values.

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

    public void initBrakeMode() {
        setMotorIdleMode(IdleMode.kBrake);
    }

    public void initCoastMode() {
        setMotorIdleMode(IdleMode.kCoast);
    }

    public void toggleIdleMode() {
        for (SwerveModule m : mModules) {
            if (((CANSparkMax) m.getDriveMotor()).getIdleMode() != IdleMode.kBrake)
                    ((CANSparkMax) m.getDriveMotor()).setIdleMode(IdleMode.kCoast);
            else {
                ((CANSparkMax) m.getDriveMotor()).setIdleMode(IdleMode.kBrake);}
            //We do not want to toggle steer motor idle mode 
        }
    }

    public void setMotorIdleMode(IdleMode mode) {
        for (SwerveModule m : mModules) {
            ((CANSparkMax) m.getDriveMotor()).setIdleMode(mode);
        }
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(speeds);
    
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, kMaxVelocity);
    
        setModuleStates(targetStates);
    }
    

    public Pose2d getPose() {
        return mPoseEstimator.getCurrentPose();
    }

    public double getPitch() {
        return mNavx.getPitch() - kPitchOffset;
    }
    public double getRoll() {
        return mNavx.getRoll() - kRollOffset;
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

    public void configureHolonomic(double driveBaseRadius) {

        HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(RobotConstantsBase.SwerveDriveBase.kAutonTranslationPID, RobotConstantsBase.SwerveDriveBase.kAutonThetaPID, 
        kMaxVelocity, driveBaseRadius, new ReplanningConfig()); 
        BooleanSupplier alliance = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return DriverStation.getAlliance().get() != Alliance.Blue;
            }
        };
        AutoBuilder.configureHolonomic(this::getPose, this::resetOdometry, this::getChassisSpeeds,  this::driveRobotRelative, config, alliance, (Subsystem)this);
    
    }
 
    /**
     * @param path
     * @param reqSubsytems
     * @param driveBaseRadius in meters (for swerve) the distance between the center of the robot to the furthest module (for mecanum this is the width of the drivebase / 2)
     */
    public MustangPPSwerveControllerCommand getFollowTrajectoryCommand(PathPlannerPath path, Subsystem[] reqSubsystems, double driveBaseRadius) {
        HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig(RobotConstantsBase.SwerveDriveBase.kAutonTranslationPID, RobotConstantsBase.SwerveDriveBase.kAutonThetaPID, 
        kMaxVelocity, driveBaseRadius, new ReplanningConfig());
        BooleanSupplier alliance = new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return DriverStation.getAlliance().get() != Alliance.Blue;
            }
        };
        return new MustangPPSwerveControllerCommand(path, this::getPose, this::getChassisSpeeds, this::driveRobotRelative, config, alliance, reqSubsystems);
    }
}
