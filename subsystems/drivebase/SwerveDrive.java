package frc.team670.mustanglib.subsystems.drivebase;

import java.util.Map;

import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team670.mustanglib.RobotConstantsBase;
// import frc.team670.mustanglib.subsystems.VisionSubsystemBase;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDriveIO.SwerveDriveIOInputs;
// import frc.team670.mustanglib.subsystems.VisionSubsystemBase;

import frc.team670.mustanglib.swervelib.Mk4iSwerveModuleHelper.GearRatio;
import frc.team670.mustanglib.swervelib.pathplanner.MustangPPSwerveControllerCommand;
import frc.team670.mustanglib.utils.SwervePoseEstimatorBase;
import org.littletonrobotics.junction.Logger;
/**
 * Swerve Drive subsystem with pose estimation.
 * 
 * @author Tarini, Edward, Justin, Ethan C, Armaan, Aditi
 */
public abstract class SwerveDrive extends DriveBase {
    protected SwerveDriveIOInputs inputs;
    protected SwerveDriveIO io;
    protected SwervePoseEstimatorBase mPoseEstimator;
    
    // private VisionSubsystemBase mVision;

    
    private final SwerveDriveKinematics kKinematics;
    
    private Rotation2d mDesiredHeading = null; // for rotation snapping

    // private final double kMaxVelocity, kMaxVoltage;
    public SwerveDrive(SwerveDriveIO io, LoggableInputs inputs) {
        super(io,inputs);
        this.io = io;
        this.inputs =(SwerveDriveIOInputs)inputs;
        


        kKinematics = new SwerveDriveKinematics(
                // Front left
                new Translation2d(io.getDriveBaseTrackWidth() / 2.0,
                        io.getDriveBaseWheelBase() / 2.0),
                // Front right
                new Translation2d(io.getDriveBaseTrackWidth() / 2.0,
                        -io.getDriveBaseWheelBase() / 2.0),
                // Back left
                new Translation2d(-io.getDriveBaseTrackWidth() / 2.0,
                        io.getDriveBaseWheelBase() / 2.0),
                // Back right
                new Translation2d(-io.getDriveBaseTrackWidth()/ 2.0,
                        -io.getDriveBaseWheelBase() / 2.0));

        // mChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        // odometer = new SwerveDriveOdometry(getSwerveKinematics(), new Rotation2d(0),
        // getModulePositions());
        // mPoseEstimator = new SwervePoseEstimatorBase(this);
        initPoseEstimator();
    
    }
    protected abstract void initPoseEstimator();

    public void drive(ChassisSpeeds chassisSpeeds) {
        io.setModuleStates(kKinematics.toSwerveModuleStates(chassisSpeeds));
    }

    public void stop() {
        drive(new ChassisSpeeds());
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the
     * robot is
     * currently facing to the 'forwards' direction.
     */
    public void resetHeading() {
        io.resetHeading((LoggableInputs)inputs);
    }
    public GearRatio getGearRatio(){
        return io.getGearRatio();
    }
    public double getMaxVelocityMetersPerSecond(){
        return io.getMaxVelocity();
    }
    
    public double getMaxAngularVelocityMetersPerSecond(){
        return io.getMaxAngularVelocity();
    }

    public void setGyroscopeRotation(Rotation2d rot) {
        io.setGyroscopeRotation(rot);
    }

    public SwerveDriveKinematics getSwerveKinematics() {
        return kKinematics;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kKinematics.toChassisSpeeds(getModuleStates());
    }

    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromRadians(inputs.gyroYawRad);
    }
    public Rotation2d getGyroscopeRotation(boolean offset) {
        return Rotation2d.fromRadians(offset?inputs.gyroYawRad:inputs.gyroYawRadUnadjusted);
    }

    public void setDesiredHeading(Rotation2d rot) {
        this.mDesiredHeading = rot;
    }

    public Rotation2d getDesiredHeading() {
        return this.mDesiredHeading;
    }
    // public double getMaxVelocityMetersPerSecond(){
    //     return  5676.0 / 60.0
    //     * kModuleConfig.getDriveReduction() * kModuleConfig.getWheelDiameter() * Math.PI;
    // }



    @Override
    public void mustangPeriodic() {
        io.checkPitchOffsetSet();
        io.checkCalibration((LoggableInputs)inputs);
        

        // if (mVision != null) {
        //     if (mPoseEstimator.getVision() == null) {
        //         mPoseEstimator.initVision(mVision);
        //     }
        // }
        mPoseEstimator.update();
        // SmartDashboard.putNumber("navX heading", getPose().getRotation().getDegrees());
        // SmartDashboard.putNumber("pitch", getPitch());
        Logger.getInstance().recordOutput("Pose2d", getPose());
        
    }

    // public void initVision(VisionSubsystemBase vision) {
    //     this.mVision = vision;
    // }

    public SwervePoseEstimatorBase getPoseEstimator() {
        return mPoseEstimator;
    }

    public void setModuleStates(SwerveModuleState[] states) {
        io.setModuleStates(states);
        
    }

    public void realignModules() {
        io.realignModules();
    }

    public void initBrakeMode() {
        io.initBrakeMode();
    }

    public void initCoastMode() {
        io.initCoastMode();
    }

    public void toggleIdleMode() {
        io.toggleIdleMode();
        
    }

    public void setMotorIdleMode(IdleMode mode) {
        io.setMotorIdleMode(mode);
    }

    public Pose2d getPose() {
        return mPoseEstimator.getCurrentPose();
    }

    public double getPitch() {
        return inputs.gyroPitchRad;
    }

    public void resetOdometry(Pose2d pose) {
        
        mPoseEstimator.setCurrentPose(pose);
    }



    public SwerveModulePosition[] getModulePositions() {
        return inputs.positions;
    }

    public SwerveModuleState[] getModuleStates() {
        return inputs.states;
    }

    public void park() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        // needs the 0.1 or else it won't even rotate the wheels
        states[0] = new SwerveModuleState(0.1, new Rotation2d(Math.PI / 4)); // front right
        states[1] = new SwerveModuleState(0.1, new Rotation2d(-Math.PI / 4)); // front left
        states[2] = new SwerveModuleState(0.1, new Rotation2d(-Math.PI / 4)); // back left
        states[3] = new SwerveModuleState(0.1, new Rotation2d(Math.PI / 4)); // back right
        io.setModuleStates(states);
    }

    public SwerveAutoBuilder getAutoBuilderFromEvents(Map<String, Command> eventMap) {
        return new SwerveAutoBuilder(this::getPose, this::resetOdometry, kKinematics,
        RobotConstantsBase.SwerveDriveBase.kAutonTranslationPID,
        RobotConstantsBase.SwerveDriveBase.kAutonThetaPID, this::setModuleStates, eventMap, true,
                new Subsystem[] { this });
    }

    public MustangPPSwerveControllerCommand getFollowTrajectoryCommand(PathPlannerTrajectory traj) {
        return new MustangPPSwerveControllerCommand(traj, this::getPose, getSwerveKinematics(),
        RobotConstantsBase.SwerveDriveBase.xController, RobotConstantsBase.SwerveDriveBase.yController,
        RobotConstantsBase.SwerveDriveBase.thetaController, this::setModuleStates,
                new Subsystem[] { this });
    }
}