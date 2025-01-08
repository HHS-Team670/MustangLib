package frc.team670.mustanglib.utils.motorcontroller;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.controls.ControlRequest;

/**
 * Wrapper class for a SparkMAX for reducing CAN bus overhead by skipping
 * duplicate set commands.
 * 
 * @author ctychen, lakshbhambhani
 */
public final class CTRELite extends TalonFX {

    protected double lastSet = Double.NaN;
    protected ControlRequest lastControlMode = null;
    protected TalonFX leader = null;
    protected MotorConfig.Motor_Type motorType = null;

    /**
     * Creates a SparkMAX on a given ID, which is controlling a specified kind of motor.
     * 
     * @param type type Which specific motor this controller will
     *                               be using. For example, NEO or BAG
     */
    public CTRELite(int id, MotorConfig.Motor_Type type) {
        super(id);
        this.motorType = type;
    }

    public double getLastSet() {
        return this.lastSet;
    }

    public ControlRequest getLastControlMode() {
        return this.lastControlMode;
    }

    public MotorConfig.Motor_Type getMotor(){
        return this.motorType;
    }
    

    /**
     * Applicable if this SparkMAX is set as a follower.
     * 
     * @return the 'leader' controller that the SparkMAX follows
     */
    public TalonFX getLeader() {
        return this.leader;
    }

    /**
     * @param leader the SparkMAX for this controller to follow, if applicable.
     */
    public void setFollow(TalonFX leader) {
        this.leader = leader;
        super.setControl(new Follower(leader.getDeviceID(), false));
    }

    /**
     * @param kdutycycle mode for this motor controller
     * @param value      value output of the controller, for the appropriate mode
     */
    public void set(ControlRequest kdutycycle, double value) {
        if (value != lastSet || kdutycycle != lastControlMode) {
            if(!isErrored()){
                this.lastSet = value;
                super.set(value);
                this.setControl(kdutycycle);
            }
        }
    }

    public void set(double value) {
        if (value != lastSet) {
            this.lastSet = value;
            super.set(value);
        }
    }

    /**
     * 
     * @return true if there is an issue with this SparkMax, false if the SparkMax
     *         is connected successfully and without errors.
     */
    public boolean isErrored() {
        return (this == null || this.isConnected() == false);
    }


}
