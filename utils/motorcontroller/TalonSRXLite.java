package frc.team670.mustanglib.utils.motorcontroller;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * Wrapper class for a TalonSRX for reducing CAN bus overhead by skipping
 * duplicate set commands.
 * 
 * @author ctychen
 */

public class TalonSRXLite extends TalonSRX {

    protected double lastSet = Double.NaN;
    protected ControlMode lastControlMode = null;

    public TalonSRXLite(int id) {
        super(id);
    }

    public double getLastSet() {
        return this.lastSet;
    }

    public ControlMode getLastMode() {
        return this.lastControlMode;
    }

    @Override
    /**
     * @param ControlMode mode for this motor controller
     * @param double      value output of the controller, for the appropriate mode
     */
    public void set(ControlMode mode, double value) {
        if (value != lastSet || mode != lastControlMode) {
            lastSet = value;
            lastControlMode = mode;
            super.set(mode, value);
        }
    }

    /**
     * 
     * @return true if there is an issue with this controller, false if it is
     *         connected successfully and without errors
     */
    public boolean isPhoenixControllerErrored() {
        return (this == null || this.getLastError() != ErrorCode.OK);
    }

}