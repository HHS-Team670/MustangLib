package frc.team670.mustanglib.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import frc.team670.mustanglib.utils.functions.MathUtils;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;

/**
 * Superclass for any rotating subsystem which uses a SparkMax to control the rotator.
 */
public abstract class SparkMaxRotatingSubsystem extends MustangSubsystemBase
        implements TunableSubsystem {

    protected SparkMAXLite mRotator;
    protected RelativeEncoder mEncoder;
    protected SparkMaxPIDController mController;
    protected double mSetpoint;
    protected double mTempSetpoint;

    protected final Config kConfig;
    protected static final double kNoSetPoint = 9999;

    public record Config(int kDeviceID, int kSlot, MotorConfig.Motor_Type kMotorType,
            IdleMode kIdleMode, double kRotatorGearRatio, double kP, double kI, double kD,
            double kFF, double kIz, double kMaxOutput, double kMinOutput, double kMaxRotatorRPM,
            double kMinRotatorRPM, double kMaxAcceleration, double kAllowedError,
            float[] kSoftLimits, int kContinuousCurrent, int kPeakCurrent) {
    }

    public SparkMaxRotatingSubsystem(Config kConfig) {
        this.kConfig = kConfig;

        this.mRotator = SparkMAXFactory.buildFactorySparkMAX(kConfig.kDeviceID, kConfig.kMotorType);
        this.mEncoder = mRotator.getEncoder();
        this.mRotator.setIdleMode(kConfig.kIdleMode);
        this.mController = mRotator.getPIDController();

        // set PID coefficients
        mController.setP(kConfig.kP);
        mController.setI(kConfig.kI);
        mController.setD(kConfig.kD);
        mController.setIZone(kConfig.kIz);
        mController.setFF(kConfig.kFF);
        mController.setOutputRange(kConfig.kMinOutput, kConfig.kMaxOutput);

        mController.setSmartMotionMaxVelocity(kConfig.kMaxRotatorRPM, kConfig.kSlot);
        mController.setSmartMotionMinOutputVelocity(kConfig.kMinRotatorRPM, kConfig.kSlot);
        mController.setSmartMotionMaxAccel(kConfig.kMaxAcceleration, kConfig.kSlot);
        mController.setSmartMotionAllowedClosedLoopError(kConfig.kAllowedError, kConfig.kSlot);

        mRotator.setSmartCurrentLimit(kConfig.kPeakCurrent, kConfig.kContinuousCurrent);

        if (kConfig.kSoftLimits == null || kConfig.kSoftLimits.length > 2) {
            mRotator.enableSoftLimit(SoftLimitDirection.kForward, false);
            mRotator.enableSoftLimit(SoftLimitDirection.kReverse, false);
        } else {
            mRotator.setSoftLimit(SoftLimitDirection.kForward, kConfig.kSoftLimits[0]);
            mRotator.setSoftLimit(SoftLimitDirection.kReverse, kConfig.kSoftLimits[1]);
            mRotator.enableSoftLimit(SoftLimitDirection.kForward, true);
            mRotator.enableSoftLimit(SoftLimitDirection.kReverse, true);
        }
        // getMaxSubsystemRPM(config.kMaxRotatorRPM);
        mSetpoint = kNoSetPoint;

        clearSetpoint();

    }

    /**
     * 
     * @return The count, in motor rotations, from the subsystem's rotator's integrated encoder.
     */
    public double getUnadjustedPosition() {
        return this.mEncoder.getPosition();
    }

    public double getMaxSubsystemRPM(double rotRPM) {
        return rotRPM / kConfig.kRotatorGearRatio;
    }

    /**
     * Sets the system's overall target position and moves to it.
     * 
     * @param setpoint The target position for this subsystem, in motor rotations
     */
    protected void setSystemMotionTarget(double setpoint) {
        setSystemMotionTarget(setpoint, 0);
    }

    /**
     * 
     * @param setpoint
     */
    protected void setSystemMotionTarget(double setpoint, double arbitraryFF) {
        if (setpoint != kNoSetPoint) {
            mController.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion, 0,
                    arbitraryFF);

        } else {
            mController.setReference(0, CANSparkMax.ControlType.kDutyCycle);
        }
        this.mSetpoint = setpoint;
    }

    /**
     * Sets an intermediate or temporary goal for the subsystem to move to. Use this when you need
     * to do something in the process of moving to your system's target, like if you need to unjam
     * something.
     * 
     * @param setpoint The temporary setpoint for the system, in motor rotations
     */
    protected void setTemporaryMotionTarget(double setpoint) {
        mTempSetpoint = setpoint;
        mController.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
    }

    /**
     * Sets the overall target angle this subsystem should move to, in degrees. In some process,
     * this is where you ultimately want to end up, so this value will be saved as the system
     * setpoint.
     * 
     * @param angle The target angle this subsystem should turn to, in degrees
     */
    public void setSystemTargetAngleInDegrees(double angle) {

        setSystemMotionTarget(getMotorRotationsFromAngle(angle));
    }

    /**
     * Sets a temporary angle setpoint for this subsystem to turn to. Use this as an intermediate
     * step in some process -- this setpoint won't be saved.
     * 
     * @param angle The angle this subsystem should turn to, in degrees
     */
    public void setTemporaryTargetAngleInDegrees(double angle) {
        setTemporaryMotionTarget(getMotorRotationsFromAngle(angle));
    }

    /**
     * 
     * @param angle The angle, in degrees, to be converted to motor rotations
     * @return The number of motor rotations, as measured by the encoder, equivalent to the
     *         subsystem turning through this angle
     */
    protected double getMotorRotationsFromAngle(double angle) {
        double rotations = (angle / 360) * kConfig.kRotatorGearRatio
                + ((int) (getUnadjustedPosition() / kConfig.kRotatorGearRatio))
                        * kConfig.kRotatorGearRatio;
        // Logger.consoleLog("Indexer motor rotations from angle is %s", rotations);
        return rotations;
    }

    /**
     * Change the system's feedforward and max velocity and acceleration temporarily. Possibly
     * useful when zeroing, testing, or unjamming.
     * 
     * @param factor Multiplier for ff. For example, if you want to halve it, factor should be 0.5
     */
    protected void temporaryScaleSmartMotionMaxVelAndAccel(double factor) {
        mController.setFF(kConfig.kFF * factor);
        mController.setSmartMotionMaxVelocity(kConfig.kMaxRotatorRPM * factor, kConfig.kSlot);
        mController.setSmartMotionMaxAccel(kConfig.kMaxAcceleration * factor, kConfig.kSlot);
    }

    /**
     * Resets feedforward, SmartMotion acceleration and velocity settings to the defined system
     * constants. Possibly useful when the system previously temporarily scaled these values for
     * testing, unjamming, or zeroing, to bring motion back to normal.
     */
    protected void resetSmartMotionSettingsToSystem() {
        mController.setFF(kConfig.kFF);
        mController.setSmartMotionMaxVelocity(kConfig.kMaxRotatorRPM, kConfig.kSlot);
        mController.setSmartMotionMaxAccel(kConfig.kMaxAcceleration, kConfig.kSlot);
    }

    /**
     * 
     * @return The current position of the subsystem, in degrees.
     */
    public double getCurrentAngleInDegrees() {
        double rotations = getRotatorEncoder().getPosition();
        double angle = 360 * ((rotations) / kConfig.kRotatorGearRatio);
        return angle;
    }

    /**
     * Calculated voltage using VoltageCalculator
     * 
     * @param voltage
     */
    public void updateArbitraryFeedForward(double voltage) {
        if (mSetpoint != kNoSetPoint) {
            mController.setReference(mSetpoint, CANSparkMax.ControlType.kSmartMotion,
                    kConfig.kSlot, voltage);
        }
    }

    /**
     * 
     * @return The setpoint in motor rotations
     */
    public double getSetpoint() {
        return mSetpoint;
    }

    /**
     * 
     * @return true if the subsystem is close to its target position, within some margin of error.
     */
    public boolean hasReachedTargetPosition() {
        return (MathUtils.doublesEqual(mEncoder.getPosition(), mSetpoint, kConfig.kAllowedError));
    }

    protected void enableCoastMode() {
        mRotator.setIdleMode(IdleMode.kCoast);
    }

    protected void enableBrakeMode() {
        mRotator.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Stops the motion of the subsystem.
     */
    public synchronized void stop() {
        mRotator.set(0);
    }

    /**
     * Clears the setpoint of this subsystem
     */
    public void clearSetpoint() {
        mController.setReference(0, CANSparkMax.ControlType.kDutyCycle);
        mSetpoint = kNoSetPoint;
    }

    public SparkMAXLite getmRotator() {
        return this.mRotator;
    }

    public RelativeEncoder getRotatorEncoder() {
        return this.mEncoder;
    }

    public SparkMaxPIDController getRotatorController() {
        return this.mController;
    }

    public void moveByPercentOutput(double output) {
        mRotator.set(output);
    }
}
