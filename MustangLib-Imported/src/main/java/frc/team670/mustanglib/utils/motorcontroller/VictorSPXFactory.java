package frc.team670.mustanglib.utils.motorcontroller;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;

import com.ctre.phoenix.ParamEnum;

/**
 * Utility class for configuring a VictorSPX to default settings and resetting to
 * factory defaults.
 * 
 * @author ruchidixit
 */
public class VictorSPXFactory {

    public static final int TIMEOUT_MS = 100;

    public static class Config {

        public NeutralMode NEUTRAL_MODE = NeutralMode.Coast;
        public double NEUTRAL_DEADBAND = 0.04; // factory default

        public boolean INVERTED = false;

        public int CONTROL_FRAME_PERIOD_MS = 5;
        public int MOTION_CONTROL_FRAME_PERIOD_MS = 100;
        public int GENERAL_STATUS_FRAME_RATE_MS = 5;
        public int FEEDBACK_STATUS_FRAME_RATE_MS = 100;
        public int QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
        public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
        public int PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;

        public double OPEN_LOOP_RAMP_RATE = 0.0;
        public double CLOSED_LOOP_RAMP_RATE = 0.0;

        public VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD = VelocityMeasPeriod.Period_100Ms;
        public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

        public boolean ENABLE_CURRENT_LIMIT = true;

        public boolean ENABLE_SOFT_LIMIT = false;
        public boolean ENABLE_LIMIT_SWITCH = false;
    }

    public static final Config defaultConfig = new Config();
    public static final Config defaultFollowerConfig = new Config();

    static {
        defaultFollowerConfig.CONTROL_FRAME_PERIOD_MS = 100;
        defaultFollowerConfig.MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
        defaultFollowerConfig.GENERAL_STATUS_FRAME_RATE_MS = 1000;
        defaultFollowerConfig.FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
        defaultFollowerConfig.QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
        defaultFollowerConfig.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
        defaultFollowerConfig.PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;
    }

    public static VictorSPXLite buildFactoryVictorSPX(int deviceID, boolean invert) {
        return buildVictorSPX(deviceID, defaultConfig, invert);
    }

    public static VictorSPXLite buildVictorSPX(int deviceID, Config config, boolean invert) {
        VictorSPXLite victorspx = new VictorSPXLite(deviceID);
        victorspx.set(ControlMode.PercentOutput, 0.0);
        victorspx.setInverted(invert);

        victorspx.changeMotionControlFramePeriod(config.MOTION_CONTROL_FRAME_PERIOD_MS);
        victorspx.clearMotionProfileHasUnderrun(TIMEOUT_MS);
        victorspx.clearMotionProfileTrajectories();

        victorspx.clearStickyFaults(TIMEOUT_MS);

        // Turn off re-zeroing by default.
        victorspx.configSetParameter(ParamEnum.eClearPositionOnLimitF, 0, 0, 0, TIMEOUT_MS);
        victorspx.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, TIMEOUT_MS);

        victorspx.configNominalOutputForward(0, TIMEOUT_MS);
        victorspx.configNominalOutputReverse(0, TIMEOUT_MS);
        victorspx.configNeutralDeadband(config.NEUTRAL_DEADBAND, TIMEOUT_MS);

        victorspx.configPeakOutputForward(1.0, TIMEOUT_MS);
        victorspx.configPeakOutputReverse(-1.0, TIMEOUT_MS);

        victorspx.setNeutralMode(config.NEUTRAL_MODE);

        victorspx.configForwardSoftLimitEnable(config.ENABLE_SOFT_LIMIT, TIMEOUT_MS);

        victorspx.configReverseSoftLimitEnable(config.ENABLE_SOFT_LIMIT, TIMEOUT_MS);

        victorspx.overrideSoftLimitsEnable(config.ENABLE_SOFT_LIMIT);

        victorspx.configVoltageCompSaturation(12); // "full output" will now scale to 12 Volts
        victorspx.enableVoltageCompensation(true); 

        victorspx.selectProfileSlot(0, 0);

        victorspx.configVelocityMeasurementPeriod(config.VELOCITY_MEASUREMENT_PERIOD, TIMEOUT_MS);
        victorspx.configVelocityMeasurementWindow(config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW, TIMEOUT_MS);

        victorspx.configOpenloopRamp(config.OPEN_LOOP_RAMP_RATE, TIMEOUT_MS);
        victorspx.configClosedloopRamp(config.CLOSED_LOOP_RAMP_RATE, TIMEOUT_MS);
        victorspx.setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS);

        return victorspx;
    }

}