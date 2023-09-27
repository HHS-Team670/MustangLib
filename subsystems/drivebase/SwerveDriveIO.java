package frc.team670.mustanglib.subsystems.drivebase;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.RobotConstantsBase;
import frc.team670.mustanglib.dataCollection.sensors.NavX;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.swervelib.Mk4ModuleConfiguration;
import frc.team670.mustanglib.swervelib.Mk4iSwerveModuleHelper;
import frc.team670.mustanglib.swervelib.Mk4iSwerveModuleHelper.GearRatio;
import frc.team670.mustanglib.swervelib.SwerveModule;


public  class SwerveDriveIO extends DriveBaseIO {
    private final NavX mNavx;
    private final SwerveModule[] mModules;
    private Config kConfig;
    private final Mk4ModuleConfiguration kModuleConfig = new Mk4ModuleConfiguration();
    private double kPitchOffset;
    private final double kMaxVelocity, kMaxVoltage;
    private Rotation2d mGyroOffset = new Rotation2d();


    public static record Config(double kDriveBaseTrackWidth, double kDriveBaseWheelBase,
            double kMaxVelocity, double kMaxAngularVelocity, double kMaxVoltage, double kMaxDriveCurrent,
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

    public SwerveDriveIO(Config config) {
        kConfig = config;
        kMaxVelocity = config.kMaxVelocity;
        kMaxVoltage = config.kMaxVoltage;

        kModuleConfig.setNominalVoltage(kMaxVoltage);
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

        mNavx = new NavX(config.kNavXPort);

        // mChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        // odometer = new SwerveDriveOdometry(getSwerveKinematics(), new Rotation2d(0),
        // getModulePositions());
        // mPoseEstimator = new SwervePoseEstimatorBase(this);
        // initPoseEstimator();
        
        kPitchOffset = RobotConstantsBase.SwerveDriveBase.kNoPitchCode;
        SmartDashboard.putNumber("MAX VELOCITY M/S", config.kMaxVelocity);
        
    }

    // protected abstract void initPoseEstimator();

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the
     * robot is
     * currently facing to the 'forwards' direction.
     */

    public void resetHeading(LoggableInputs inputs) {
        
        mGyroOffset = Rotation2d.fromRotations(((SwerveDriveIOInputs)inputs).gyroYawRadUnadjusted);
    }



    public void setGyroscopeRotation(Rotation2d rot) {
        mGyroOffset = rot;
    }
    public void checkPitchOffsetSet(){
        if(kPitchOffset == RobotConstantsBase.SwerveDriveBase.kNoPitchCode){
            kPitchOffset = mNavx.getPitch();
        }
    }


    // public double getMaxVelocityMetersPerSecond(){
    // return 5676.0 / 60.0
    // * kModuleConfig.getDriveReduction() * kModuleConfig.getWheelDiameter() *
    // Math.PI;
    // }
    @Override
    public HealthState checkHealth(){
        for (SwerveModule curr : getModules()) {
            CANSparkMax motor = (CANSparkMax) curr.getDriveMotor();
            if (motor.getLastError() != REVLibError.kOk) {
                SmartDashboard.putString("Swerve Module " + motor.getDeviceId() + " ERROR:",
                        motor.getLastError().toString());
                return HealthState.RED;
            }
        }
        return HealthState.GREEN;
    }

    public void checkCalibration(LoggableInputs inputs) {
        if (mGyroOffset == null && !mNavx.isCalibrating()) {
            resetHeading(inputs);
            realignModules();
        }
    }

    public double getMaxVelocity() {
        return kMaxVelocity;
    }
    
    public double getMaxAngularVelocity() {
        return kConfig.kMaxAngularVelocity;
    }

    public double getDriveBaseTrackWidth(){
        return kConfig.kDriveBaseTrackWidth;
    }

    public double getDriveBaseWheelBase(){
        return kConfig.kDriveBaseWheelBase;
    }

    public GearRatio getGearRatio(){
        return kConfig.kSwerveModuleGearRatio;
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
            if (m.getDriveMotor().getIdleMode() == IdleMode.kBrake)
                m.getDriveMotor().setIdleMode(IdleMode.kCoast);
            else {
                m.getDriveMotor().setIdleMode(IdleMode.kBrake);
            }
            // We do not want to toggle steer motor idle mode
        }
    }

    public void setMotorIdleMode(IdleMode mode) {
        for (SwerveModule m : mModules) {
            (m.getDriveMotor()).setIdleMode(mode);
        }
    }

    public void resetOdometry(Pose2d pose,SwerveDriveIOInputs inputs) {
        SwerveModulePosition[] zeroedPos = new SwerveModulePosition[4];
        for (int i = 0; i < zeroedPos.length; i++) {
            zeroedPos[i] = new SwerveModulePosition();
        }
        setGyroscopeRotation(
                Rotation2d.fromRadians(inputs.gyroYawRadUnadjusted + pose.getRotation().getRadians()));

    }

    public SwerveModule[] getModules() {
        return mModules;
    }
    
  

 



    @Override
    public void updateInputs(LoggableInputs inputs) {
        SwerveDriveIOInputs input = (SwerveDriveIOInputs) inputs;
        for (int i = 0; i < mModules.length; i++) {
            // input.realModuleRotations[i] = mModules[i].getSteerAngle();
            // input.realModuleVelocities[i] = mModules[i].getDriveVelocity();
            input.states[i] = mModules[i].getState();
            input.positions[i] = mModules[i].getPosition();

        }
        input.gyroYawRad = getGyroYawRad(true).getRadians();
        input.gyroPitchRad = Units.degreesToRadians(mNavx.getPitch() - kPitchOffset);
        input.gyroYawRadUnadjusted = getGyroYawRad(false).getRadians();
        // input.updateFromSuper();
    }

    private Rotation2d getGyroYawRad(boolean offset) {
        if (mNavx.isMagnetometerCalibrated()) {
            // We will only get valid fused headings if the magnetometer is calibrated
            if (offset) {
                Rotation2d angle = Rotation2d.fromDegrees(-mNavx.getFusedHeading()).minus(mGyroOffset);
           
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
    public void debugOutputs() {

    }

    @AutoLog
    public static class SwerveDriveIOInputs extends DriveBaseIOInputs {
        // public double[] realModuleRotations = new double[4];
        // public double[] realModuleVelocities = new double[4];
        public SwerveModuleState[] states = new SwerveModuleState[4];
        public SwerveModulePosition[] positions = new SwerveModulePosition[4];
        public double gyroYawRad;
        public double gyroPitchRad;
        public double gyroYawRadUnadjusted;

    }

}
