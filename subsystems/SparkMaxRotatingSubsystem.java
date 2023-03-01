package frc.team670.mustanglib.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import frc.team670.mustanglib.utils.PIDConstantSet;
import frc.team670.mustanglib.utils.functions.MathUtils;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;

/**
 * Superclass for any rotating subsystem which uses a SparkMax to control the
 * rotator.
 * 
 * @author ctychen
 */
public abstract class SparkMaxRotatingSubsystem extends MustangSubsystemBase implements TunableSubsystem {

    protected SparkMAXLite rotator;
    protected RelativeEncoder rotator_encoder;
    protected SparkMaxPIDController rotator_controller;
    protected double setpoint;
    protected double tempSetpoint;
    protected PIDConstantSet PIDs;
    protected double MAX_OUTPUT, MIN_OUTPUT;
    protected double MAX_ROT_RPM, MAX_SUB_RPM, MIN_RPM, MAX_ACC, ALLOWED_ERR;
    protected int SMARTMOTION_SLOT;
    protected double ROTATOR_GEAR_RATIO;
    protected static final double NO_SETPOINT = 9999;


    /**
     * Configuration for this RotatingSubsystem's properties. Use this to keep track
     * of PID and SmartMotion constants
     */
    public static abstract class Config {

        public abstract int getDeviceID();

        public abstract int getSlot();

        public abstract MotorConfig.Motor_Type getMotorType();

        public abstract IdleMode setRotatorIdleMode();

        public abstract double getRotatorGearRatio();

        public abstract double getP();

        public abstract double getI();

        public abstract double getD();

        public abstract double getFF();

        public abstract double getIz();

        public abstract double getMaxOutput();

        public abstract double getMinOutput();

        public abstract double getMaxRotatorRPM();

        public abstract double getMinRotatorRPM();

        public abstract double getMaxAcceleration();

        public abstract double getAllowedError();

        /**
         * @return An array of soft limits, in rotations, for this system:
         *         [forwardLimit, reverseLimit].
         * @return null if this system does not have soft limits.
         */
        public abstract float[] getSoftLimits();

        public abstract int getContinuousCurrent();

        public abstract int getPeakCurrent();

        public float convertDegreesToRotations(float d) {
            return (float) ((d / 360) * getRotatorGearRatio());
        }
    }

    public SparkMaxRotatingSubsystem(Config config) {
        this.rotator = SparkMAXFactory.buildFactorySparkMAX(config.getDeviceID(), config.getMotorType());
        this.rotator_encoder = rotator.getEncoder();
        this.rotator.setIdleMode(config.setRotatorIdleMode());
        this.rotator_controller = rotator.getPIDController();

        this.ROTATOR_GEAR_RATIO = config.getRotatorGearRatio();

        // PID coefficients
        this.PIDs = new PIDConstantSet(config.getP(), config.getI(), config.getIz(), config.getD(), config.getFF());
        this.MAX_OUTPUT = config.getMaxOutput();
        this.MIN_OUTPUT = config.getMinOutput();

        // Smart Motion Coefficients
        this.MAX_ROT_RPM = config.getMaxRotatorRPM(); // rpm
        this.MAX_SUB_RPM = getMaxSubsystemRPM(config.getMaxRotatorRPM());
        this.MAX_ACC = config.getMaxAcceleration();
        this.ALLOWED_ERR = config.getAllowedError();

        // set PID coefficients
        this.rotator_controller.setP(PIDs.P);
        this.rotator_controller.setI(PIDs.I);
        this.rotator_controller.setD(PIDs.D);
        this.rotator_controller.setIZone(PIDs.I_ZONE);
        this.rotator_controller.setFF(PIDs.FF);
        this.rotator_controller.setOutputRange(this.MIN_OUTPUT, this.MAX_OUTPUT);

        this.SMARTMOTION_SLOT = config.getSlot();
        rotator_controller.setSmartMotionMaxVelocity(this.MAX_ROT_RPM, this.SMARTMOTION_SLOT);
        rotator_controller.setSmartMotionMinOutputVelocity(this.MIN_RPM, this.SMARTMOTION_SLOT);
        rotator_controller.setSmartMotionMaxAccel(this.MAX_ACC, this.SMARTMOTION_SLOT);
        rotator_controller.setSmartMotionAllowedClosedLoopError(this.ALLOWED_ERR, this.SMARTMOTION_SLOT);

        rotator.setSmartCurrentLimit(config.getPeakCurrent(), config.getContinuousCurrent());

        if (config.getSoftLimits() == null || config.getSoftLimits().length > 2) {
            rotator.enableSoftLimit(SoftLimitDirection.kForward, false);
            rotator.enableSoftLimit(SoftLimitDirection.kReverse, false);
        } else {
            rotator.setSoftLimit(SoftLimitDirection.kForward, config.getSoftLimits()[0]);
            rotator.setSoftLimit(SoftLimitDirection.kReverse, config.getSoftLimits()[1]);
            rotator.enableSoftLimit(SoftLimitDirection.kForward, true);
            rotator.enableSoftLimit(SoftLimitDirection.kReverse, true);
        }
        //getMaxSubsystemRPM(config.getMaxRotatorRPM());
        setpoint = NO_SETPOINT;

        clearSetpoint();

    }

    /**
     * 
     * @return The count, in motor rotations, from the subsystem's rotator's
     *         integrated encoder.
     */
    public double getUnadjustedPosition() {
        return this.rotator_encoder.getPosition();
    }

    public double getMaxSubsystemRPM(double rotRPM){
        return rotRPM/this.ROTATOR_GEAR_RATIO;
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
        if(setpoint != NO_SETPOINT) {
            rotator_controller.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion, 0, arbitraryFF);
            
        } else {
            rotator_controller.setReference(0, CANSparkMax.ControlType.kDutyCycle);
        }
        this.setpoint = setpoint;
    }

    /**
     * Sets an intermediate or temporary goal for the subsystem to move to. Use this
     * when you need to do something in the process of moving to your system's
     * target, like if you need to unjam something.
     * 
     * @param setpoint The temporary setpoint for the system, in motor rotations
     */ 
    protected void setTemporaryMotionTarget(double setpoint) {
        tempSetpoint = setpoint;
        rotator_controller.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
    }

    /**
     * Sets the overall target angle this subsystem should move to, in degrees. In
     * some process, this is where you ultimately want to end up, so this value will
     * be saved as the system setpoint.
     * 
     * @param angle The target angle this subsystem should turn to, in degrees
     */
    public void setSystemTargetAngleInDegrees(double angle) {
        
        setSystemMotionTarget(getMotorRotationsFromAngle(angle));
    }

    /**
     * Sets a temporary angle setpoint for this subsystem to turn to. Use this as an
     * intermediate step in some process -- this setpoint won't be saved.
     * 
     * @param angle The angle this subsystem should turn to, in degrees
     */
    public void setTemporaryTargetAngleInDegrees(double angle) {
        setTemporaryMotionTarget(getMotorRotationsFromAngle(angle));
    }

    /**
     * 
     * @param angle The angle, in degrees, to be converted to motor rotations
     * @return The number of motor rotations, as measured by the encoder, equivalent
     *         to the subsystem turning through this angle
     */
    protected double getMotorRotationsFromAngle(double angle) {
        double rotations = (angle / 360) * this.ROTATOR_GEAR_RATIO
                + ((int) (getUnadjustedPosition() / this.ROTATOR_GEAR_RATIO)) * this.ROTATOR_GEAR_RATIO;
        // Logger.consoleLog("Indexer motor rotations from angle is %s", rotations);
        return rotations;
    }

    /**
     * Change the system's feedforward and max velocity and acceleration
     * temporarily. Possibly useful when zeroing, testing, or unjamming.
     * 
     * @param factor Multiplier for ff. For example, if you want to halve it, factor
     *               should be 0.5
     */
    protected void temporaryScaleSmartMotionMaxVelAndAccel(double factor) {
        rotator_controller.setFF(PIDs.FF * factor);
        rotator_controller.setSmartMotionMaxVelocity(this.MAX_ROT_RPM * factor, this.SMARTMOTION_SLOT);
        rotator_controller.setSmartMotionMaxAccel(this.MAX_ACC * factor, this.SMARTMOTION_SLOT);
    }

    /**
     * Resets feedforward, SmartMotion acceleration and velocity settings to the
     * defined system constants. Possibly useful when the system previously
     * temporarily scaled these values for testing, unjamming, or zeroing, to bring
     * motion back to normal.
     */
    protected void resetSmartMotionSettingsToSystem() {
        rotator_controller.setFF(PIDs.FF);
        rotator_controller.setSmartMotionMaxVelocity(this.MAX_ROT_RPM, this.SMARTMOTION_SLOT);
        rotator_controller.setSmartMotionMaxAccel(this.MAX_ACC, this.SMARTMOTION_SLOT);
    }

    /**
     * 
     * @return The current position of the subsystem, in degrees.
     */
    public double getCurrentAngleInDegrees() {
        double rotations = getRotatorEncoder().getPosition();
        double angle = 360 * ((rotations) / this.ROTATOR_GEAR_RATIO);
        return angle;
    }

    /**
     * 
     * @return true if the subsystem is close to its target position, within some
     *         margin of error.
     */
    public boolean hasReachedTargetPosition() {
        return (MathUtils.doublesEqual(rotator_encoder.getPosition(), setpoint, ALLOWED_ERR));
    }

    protected void enableCoastMode() {
        rotator.setIdleMode(IdleMode.kCoast);
    }

    protected void enableBrakeMode() {
        rotator.setIdleMode(IdleMode.kBrake);
    }

    /**
     * Stops the motion of the subsystem.
     */
    public synchronized void stop() {
        rotator.set(0);
    }

    /**
     * Clears the setpoint of this subsystem
     */
    public void clearSetpoint() {
        rotator_controller.setReference(0, CANSparkMax.ControlType.kDutyCycle);
        setpoint = NO_SETPOINT;
    }

    public SparkMAXLite getRotator() {
        return this.rotator;
    }

    public RelativeEncoder getRotatorEncoder() {
        return this.rotator_encoder;
    }

    public SparkMaxPIDController getRotatorController() {
        return this.rotator_controller;
    }

    public void moveByPercentOutput(double output) {
        rotator.set(output);
    }
}