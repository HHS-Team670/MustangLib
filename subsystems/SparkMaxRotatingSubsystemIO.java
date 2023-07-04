package frc.team670.mustanglib.subsystems;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import org.littletonrobotics.junction.Logger;
import frc.team670.mustanglib.utils.MustangNotifications;
import frc.team670.mustanglib.utils.functions.MathUtils;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;

public abstract class SparkMaxRotatingSubsystemIO extends MustangSubsystemBaseIO {
    protected SparkMAXLite mRotator;
    protected RelativeEncoder mEncoder;
    protected SparkMaxPIDController mController;
    public final Config kConfig;
    protected static final double kNoSetPoint = 9999;
    protected double mSetpoint;
    protected double mTempSetpoint;
    
    public record Config(int kDeviceID, int kSlot, MotorConfig.Motor_Type kMotorType,
            IdleMode kIdleMode, double kRotatorGearRatio, double kP, double kI, double kD,
            double kFF, double kIz, double kMaxOutput, double kMinOutput, double kMaxRotatorRPM,
            double kMinRotatorRPM, double kMaxAcceleration, double kAllowedErrorDegrees, 
            float[] kSoftLimits, int kContinuousCurrent, int kPeakCurrent, double kAllowedDeviation) {

        
    }


    @AutoLog
    public static class SparkMaxRotatingSubsystemIOInputs {
        double mEncoderPositionUnadjusted=0;
        double mRotatorPower=0;

    }


    public SparkMaxRotatingSubsystemIO(Config kConfig){
        this.kConfig = kConfig;

        this.mRotator = SparkMAXFactory.buildFactorySparkMAX(kConfig.kDeviceID, kConfig.kMotorType);
        this.mEncoder = mRotator.getEncoder();
        this.mRotator.setIdleMode(kConfig.kIdleMode);
        this.mController = mRotator.getPIDController();
        // this.kConfig.kAllowedDeviation = kConfig.kRotatorGearRatio * 0.2 / 360;

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
        mController.setSmartMotionAllowedClosedLoopError(kConfig.kAllowedDeviation, kConfig.kSlot);

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
    
    public void updateInputs(SparkMaxRotatingSubsystemIOInputsAutoLogged inputs){
        /**
         * The count, in motor rotations, from the subsystem's rotator's integrated encoder.
         */
        inputs.mEncoderPositionUnadjusted=this.mEncoder.getPosition();
        inputs.mRotatorPower=mRotator.get();
        


    }

    
    public Config getConfig(){
        return kConfig;
    }

    public double getMaxSubsystemRPM(double rotRPM) {
        return rotRPM / kConfig.kRotatorGearRatio;
    }

    public boolean inSoftLimits(double setpoint){
        if (kConfig.kSoftLimits != null && (setpoint > kConfig.kSoftLimits[0] || setpoint < kConfig.kSoftLimits[1])) {
            
            return false;
        }
        return true;
    }



    /**
     * 
     * @param setpoint
     */
    protected void setSystemMotionTarget(double setpoint, double arbitraryFF) {
        if (!inSoftLimits(setpoint)) {
            
            MustangNotifications.reportWarning("Improper setpoint: " + setpoint + " Setpoint should be between " +kConfig.kSoftLimits[1] + " and " + kConfig.kSoftLimits[0]);
            return;
        }
        if (setpoint != kNoSetPoint) {
            mController.setReference(setpoint, SparkMAXLite.ControlType.kSmartMotion, 0,
                    arbitraryFF);
            

        } else {
            mController.setReference(0, SparkMAXLite.ControlType.kDutyCycle);
        }
        this.mSetpoint = setpoint;
        Logger.getInstance().recordOutput("SparkMaxRotatingSubsystem/Device ID:"+kConfig.kDeviceID()+"/setpoint",setpoint);
    }

    /**
     * Sets an intermediate or temporary goal for the subsystem to move to. Use this when you need
     * to do something in the process of moving to your system's target, like if you need to unjam
     * something.
     * 
     * @param setpoint The temporary setpoint for the system, in motor rotations
     */
    protected void setTemporaryMotionTarget(double setpoint) {
        if (!inSoftLimits(setpoint)) {
        MustangNotifications.reportWarning(("Improper setpoint: " + setpoint + " Setpoint should be between " +kConfig.kSoftLimits[1]+ " and " + kConfig.kSoftLimits[0]));
            return;
        }
        mTempSetpoint = setpoint;
        mController.setReference(setpoint, SparkMAXLite.ControlType.kSmartMotion);
        Logger.getInstance().recordOutput("SparkMaxRotatingSubsystem/Device ID:"+kConfig.kDeviceID()+"/temporarySetpoint",setpoint);
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
     * Calculated voltage using VoltageCalculator
     * 
     * @param voltage
     */
    public void updateArbitraryFeedForward(double voltage) {
        if (mSetpoint != kNoSetPoint) {
            mController.setReference(mSetpoint, SparkMAXLite.ControlType.kSmartMotion, kConfig.kSlot,
                    voltage);
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
    public boolean hasReachedTargetPosition(SparkMaxRotatingSubsystemIOInputs inputs) {
        return (MathUtils.doublesEqual(inputs.mEncoderPositionUnadjusted, mSetpoint, kConfig.kAllowedDeviation));
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
        if (!inSoftLimits(0)) {
            MustangNotifications.reportWarning("Improper setpoint: " + 0 + " Setpoint should be between " +kConfig.kSoftLimits[1]
                    + " and " + kConfig.kSoftLimits[0]);
            return;
        }
        mController.setReference(0, SparkMAXLite.ControlType.kDutyCycle);
        mSetpoint = kNoSetPoint;
    }
    @Override
    public void debugOutputs(){
        Logger.getInstance().recordOutput("SparkMaxRotatingSubsystem/Device ID:"+kConfig.kDeviceID()+"/temporarySetpoint",this.mTempSetpoint);
        Logger.getInstance().recordOutput("SparkMaxRotatingSubsystem/Device ID:"+kConfig.kDeviceID()+"/temporarySetpoint",this.mSetpoint);
    }

    public SparkMAXLite getRotator() {
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


    

