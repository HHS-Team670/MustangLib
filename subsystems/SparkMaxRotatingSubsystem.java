package frc.team670.mustanglib.subsystems;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import frc.team670.mustanglib.subsystems.SparkMaxRotatingSubsystemIO.Config;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.functions.MathUtils;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;

/**
 * Superclass for any rotating subsystem which uses a SparkMax to control the rotator.
 */
public abstract class SparkMaxRotatingSubsystem extends MustangSubsystemBase
        implements TunableSubsystem {
    private SparkMaxRotatingSubsystemIO io;
    private SparkMaxRotatingSubsystemIOInputsAutoLogged inputs;


    


    
    
    // private final double kConfig.kAllowedDeviation;
    
 

    public SparkMaxRotatingSubsystem(Config kConfig,SparkMaxRotatingSubsystemIO io) {
        super(io,new SparkMaxRotatingSubsystemIOInputsAutoLogged());
        this.io=io;
        this.inputs=(SparkMaxRotatingSubsystemIOInputsAutoLogged)(super.getInputs());
    }

    /**
     * 
     * @return The count, in motor rotations, from the subsystem's rotator's integrated encoder.
     */
    public double getUnadjustedPosition() {
        return inputs.mEncoderPositionUnadjusted;
    }
    /**
     * The function calculates the maximum subsystem RPM based on the given rotator RPM and a constant
     * gear ratio.
     * 
     * @param rotRPM The parameter "rotRPM" represents the rotational speed of a subsystem, rotations per minute
     * @return The method is returning the maximum subsystem RPM.
     */
    public double getMaxSubsystemRPM(double rotRPM) {
        return rotRPM / io.kConfig.kRotatorGearRatio();
    }

    

    /**
     * The function sets the motion target for a system, but first checks if the setpoint is within the soft
     * limits and logs an error message if it is not.
     * 
     * @param setpoint The setpoint is the desired target value for the system's motion.
     * @return if this action was sucessful
     */
    protected void setSystemMotionTarget(double setpoint) {
        
        io.setSystemMotionTarget(setpoint, 0);
    }

 

    /**
     * Sets an intermediate or temporary goal for the subsystem to move to. Use this when you need
     * to do something in the process of moving to your system's target, like if you need to unjam
     * something.
     * 
     * @param setpoint The temporary setpoint for the system, in motor rotations
     * @return if this action was sucessful
     */
    protected void setTemporaryMotionTarget(double setpoint) {
        io.setTemporaryMotionTarget(setpoint);
    }

    /**
     * Sets the overall target angle this subsystem should move to, in degrees. In some process,
     * this is where you ultimately want to end up, so this value will be saved as the system
     * setpoint.
     * 
     * @param angle The target angle this subsystem should turn to, in degrees
     * @return if this action was sucessful
     */
    public void setSystemTargetAngleInDegrees(double angle) {
        
        
        setSystemMotionTarget(getMotorRotationsFromAngle(angle));
    }

    /**
     * Sets a temporary angle setpoint for this subsystem to turn to. Use this as an intermediate
     * step in some process -- this setpoint won't be saved.
     * 
     * @param angle The angle this subsystem should turn to, in degrees
     * @return if this action was sucessful
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
        double rotations = (angle / 360) * io.kConfig.kRotatorGearRatio()
                + ((int) (getUnadjustedPosition() / io.kConfig.kRotatorGearRatio()))
                        * io.kConfig.kRotatorGearRatio();
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
        io.temporaryScaleSmartMotionMaxVelAndAccel(factor);
    }

    /**
     * Resets feedforward, SmartMotion acceleration and velocity settings to the defined system
     * constants. Possibly useful when the system previously temporarily scaled these values for
     * testing, unjamming, or zeroing, to bring motion back to normal.
     */
    protected void resetSmartMotionSettingsToSystem() {
        io.resetSmartMotionSettingsToSystem();
    }

    /**
     * 
     * @return The current position of the subsystem, in degrees.
     */
    public double getCurrentAngleInDegrees() {
        double rotations = inputs.mEncoderPositionUnadjusted;
        double angle = 360 * ((rotations) / io.kConfig.kRotatorGearRatio());
        return angle;
    }

    /**
     * Calculated voltage using VoltageCalculator
     * 
     * @param voltage
     */
    public void updateArbitraryFeedForward(double voltage) {
        io.updateArbitraryFeedForward(voltage);
    }

    /**
     * 
     * @return The setpoint in motor rotations
     */
    public double getSetpoint() {
        return io.getSetpoint();
    }

    /**
     * 
     * @return true if the subsystem is close to its target position, within some margin of error.
     */
    public boolean hasReachedTargetPosition() {
         return (MathUtils.doublesEqual(inputs.mEncoderPositionUnadjusted, io.getSetpoint(), io.kConfig.kAllowedDeviation()));
    }

    /*
     * sets idle mode to coast
     */
    protected void enableCoastMode() {
        io.enableCoastMode();
    }
    /*
     * sets idle mode to brake
     */
    protected void enableBrakeMode() {
        io.enableBrakeMode();
    }

    /**
     * Stops the motion of the subsystem.
     */
    public synchronized void stop() {
        io.stop();
    }

    /**
     * Clears the setpoint of this subsystem
     * @return if this action was sucessful
     */
    public void clearSetpoint() {
        
       io.clearSetpoint();
    }
  /**
    * The function returns the SparkMAXLite object for the rotator.
    * 
    * @return The method is returning an object of type SparkMAXLite.
    */
    public SparkMAXLite getRotator() {
        return io.getRotator();
    }
   /**
     * The function returns the rotator encoder.
     * 
     * @return The method is returning an object of type RelativeEncoder.
     */
    public RelativeEncoder getRotatorEncoder() {
        return io.getRotatorEncoder();
    }
   /**
     * The function returns a SparkMaxPIDController object named "mController".
     * 
     * @return The method is returning a SparkMaxPIDController object.
     */
    public SparkMaxPIDController getRotatorController() {
        return io.getRotatorController();
    }
   /**
     * The function sets the output of a rotator based on a given percentage.
     * 
     * @param output The "output" parameter is a double value representing the desired percent output
     * for the "mRotator" object.
     */
    public void moveByPercentOutput(double output) {
        io.moveByPercentOutput(output);
    }
}
