package frc.team670.mustanglib.utils.motorcontroller;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * Wrapper class for a VictorSPX for reducing CAN bus overhead by skipping
 * duplicate set commands.
 * 
 * @author ctychen
 */

public class VictorSPXLite extends VictorSPX {

    protected double lastSet = Double.NaN;
    protected ControlMode lastControlMode = null;

    public VictorSPXLite(int id) {
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

}