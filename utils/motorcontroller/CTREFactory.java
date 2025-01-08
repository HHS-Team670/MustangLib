package frc.team670.mustanglib.utils.motorcontroller;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.team670.mustanglib.swervelib.ctre.CtreUtils;
import frc.team670.mustanglib.utils.ConsoleLogger;
import frc.team670.mustanglib.utils.MustangNotifications;

/**
 * Utility class for configuring a Kraken to default settings and resetting to
 * factory defaults.
 * 
 * @author ctychen, ruchidixit, smishra467
 */
public class CTREFactory {

    public static class Config {

        public boolean BURN_FACTORY_DEFAULT_FLASH = false;
        public NeutralModeValue DEFAULT_MODE = NeutralModeValue.Coast;
        public boolean INVERTED = false;
        public int CURRENT_LIMIT = 40;

        public int STATUS_FRAME_0_RATE_MS = 10; // TODO: set default status frame periods
        public int STATUS_FRAME_1_RATE_MS = 20;
        public int STATUS_FRAME_2_RATE_MS = 20;
        public int STATUS_FRAME_3_RATE_MS = 50;
        public int STATUS_FRAME_4_RATE_MS = 20;
        public int STATUS_FRAME_5_RATE_MS = 200;
        public int STATUS_FRAME_6_RATE_MS = 200;

        public double OPEN_LOOP_RAMP_RATE = 0.0;
        public double CLOSED_LOOP_RAMP_RATE = 0.0;

        public double NOMINAL_VOLTAGE = 12.0;

        public static CTREFactory.Config copy(CTREFactory.Config config){
            Config copy = new Config();
            copy.BURN_FACTORY_DEFAULT_FLASH = config.BURN_FACTORY_DEFAULT_FLASH;
            copy.DEFAULT_MODE = config.DEFAULT_MODE;
            copy.INVERTED = config.INVERTED;
            copy.STATUS_FRAME_0_RATE_MS = config.STATUS_FRAME_0_RATE_MS;
            copy.STATUS_FRAME_1_RATE_MS = config.STATUS_FRAME_1_RATE_MS;
            copy.STATUS_FRAME_2_RATE_MS = config.STATUS_FRAME_2_RATE_MS;
            copy.STATUS_FRAME_3_RATE_MS = config.STATUS_FRAME_3_RATE_MS;
            copy.STATUS_FRAME_4_RATE_MS = config.STATUS_FRAME_4_RATE_MS;
            copy.STATUS_FRAME_5_RATE_MS = config.STATUS_FRAME_5_RATE_MS;
            copy.STATUS_FRAME_6_RATE_MS = config.STATUS_FRAME_6_RATE_MS;
            copy.OPEN_LOOP_RAMP_RATE = config.OPEN_LOOP_RAMP_RATE;
            copy.CLOSED_LOOP_RAMP_RATE = config.CLOSED_LOOP_RAMP_RATE;
            copy.NOMINAL_VOLTAGE = config.NOMINAL_VOLTAGE;
            copy.CURRENT_LIMIT = config.CURRENT_LIMIT;

            return copy;            

        }

    }

    public static final Config defaultConfig = new Config(); // For motors where we care about position and velocity
    public static final Config defaultVelocityConfig = new Config();// For motors were we only care about precise velocity and not precise position 
    public static final Config defaultPositionConfig = new Config(); // For motors were we only care about precise position and not precise velocity 
    public static final Config defaultLowUpdateRateConfig = new Config(); // For motors where we neither care about precise velocity or position
    public static final Config defaultFollowerConfig = new Config(); //For follower motors

    //We leave frames 0 and 1 at default for non follower motors because we often need to track motor applied output (Frame 0) and current (Frame 1) regadless of if we need precise position or velocity
    static {
        defaultFollowerConfig.STATUS_FRAME_0_RATE_MS = 30000;
        defaultFollowerConfig.STATUS_FRAME_1_RATE_MS = 30000;
        defaultFollowerConfig.STATUS_FRAME_2_RATE_MS = 30000;
        defaultFollowerConfig.STATUS_FRAME_3_RATE_MS = 30000;
        defaultFollowerConfig.STATUS_FRAME_4_RATE_MS = 30000;
        defaultFollowerConfig.STATUS_FRAME_5_RATE_MS = 30000;
        defaultFollowerConfig.STATUS_FRAME_6_RATE_MS = 30000;

        defaultVelocityConfig.STATUS_FRAME_2_RATE_MS = 30000;
        defaultVelocityConfig.STATUS_FRAME_3_RATE_MS = 30000;
        defaultVelocityConfig.STATUS_FRAME_4_RATE_MS = 30000;
        defaultVelocityConfig.STATUS_FRAME_5_RATE_MS = 30000;
        defaultVelocityConfig.STATUS_FRAME_6_RATE_MS = 30000;

        defaultPositionConfig.STATUS_FRAME_3_RATE_MS = 30000;
        defaultPositionConfig.STATUS_FRAME_4_RATE_MS = 30000;
        defaultPositionConfig.STATUS_FRAME_5_RATE_MS = 30000;
        defaultPositionConfig.STATUS_FRAME_6_RATE_MS = 30000;
        
        defaultLowUpdateRateConfig.STATUS_FRAME_2_RATE_MS = 30000;
        defaultLowUpdateRateConfig.STATUS_FRAME_3_RATE_MS = 30000;
        defaultLowUpdateRateConfig.STATUS_FRAME_4_RATE_MS = 30000;
        defaultLowUpdateRateConfig.STATUS_FRAME_5_RATE_MS = 30000;
        defaultLowUpdateRateConfig.STATUS_FRAME_6_RATE_MS = 30000;
    }

    /**
     * Creates a CTRELite with factory settings.
     */
    public static CTRELite buildFactoryKraken(int deviceID, MotorConfig.Motor_Type motorType) {
        return buildKraken(deviceID, defaultConfig, motorType);
    }



    public static CTRELite setPermanentFollower(int deviceID, CTRELite leader) {
        return setPermanentFollower(deviceID, leader, false);
    }

    public static CTRELite setPermanentFollower(int deviceID, CTRELite leader, boolean inverted) {
        CTRELite kraken = buildKraken(deviceID, defaultFollowerConfig, leader.getMotor());
        kraken.setControl(new Follower(leader.getDeviceID(), inverted));
        return kraken;
    }
 
    /**
     * 
     * @param deviceID  CAN ID of this kraken
     * @param config    The configuration to set this for, ex. default or
     *                  defaultFollower
     * @param motorType The kind of motor this controller will be using
     * @return CTRELite set to this configuration with current limit
     */
    public static CTRELite buildKraken(int deviceID, Config config, MotorConfig.Motor_Type motorType) {
        CTRELite kraken = new CTRELite(deviceID, motorType);
        // kraken.restoreFactoryDefaults();

        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

        for(int i=0;i<30;i++){
            kraken.setControl(new MotionMagicDutyCycle(0));
            kraken.setInverted(config.INVERTED);

            motorConfiguration.CurrentLimits.SupplyCurrentLimit = MotorConfig.MOTOR_MAX_CURRENT.get(motorType); // TODO: set to real values
            motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
            motorConfiguration.CurrentLimits.SupplyTimeThreshold=0.25;
            motorConfiguration.CurrentLimits.SupplyCurrentThreshold=80;
            kraken.setVoltage(12);
        }
        
        BaseStatusSignal.setUpdateFrequencyForAll(10, kraken.getStickyFaultField()); //period 0
        BaseStatusSignal.setUpdateFrequencyForAll(50, kraken.getPosition(), kraken.getVelocity(),  kraken.getRotorPosition()); //period 1
        BaseStatusSignal.setUpdateFrequencyForAll(1, kraken.getSupplyCurrent(), kraken.getStatorCurrent(), kraken.getMotorVoltage(), kraken.getSupplyVoltage()); //period 2
        
        CtreUtils.checkCtreError(kraken.getConfigurator().apply(motorConfiguration),
                            "Failed to configure Motor Controller");

        return kraken;
    }

    /**
     * Used to build a pair of spark max controllers to control motors. Creates a
     * leader on the port which is working and makes other controller follow it
     * 
     * @param motor1DeviceID The CAN ID of spark max controller 1
     * @param motor2DeviceID The CAN ID of spark max controller 2
     * @return motorPair a pair of motors with the first one as its leader and
     *         second one as the follower
     */
    public static List<CTRELite> buildFactoryKrakenPair(int motor1DeviceID, int motor2DeviceID, boolean invertFollower, MotorConfig.Motor_Type motorType) {
        return buildKrakenPair(motor1DeviceID, motor2DeviceID, invertFollower, defaultConfig, defaultFollowerConfig, motorType);
    }


    /**
     * Used to build a pair of spark max controllers to control motors. Creates a
     * leader on the port which is working and makes other controller follow it
     * 
     * @param motor1DeviceID The CAN ID of spark max controller 1
     * @param motor2DeviceID The CAN ID of spark max controller 2
     * @param config         The config to be set on to the motor controllers
     * @return motorPair a pair of motors with the first one as its leader and
     *         second one as the follower
     */
    public static List<CTRELite> buildKrakenPair(int motor1DeviceID, int motor2DeviceID, boolean invertFollower, Config config, MotorConfig.Motor_Type motorType) {
        return buildKrakenPair(motor1DeviceID, motor2DeviceID, invertFollower, config, config, motorType);
    }

    /**
     * Used to build a pair of spark max controllers to control motors. Creates a
     * leader on the port which is working and makes other controller follow it
     * 
     * @param motor1DeviceID The CAN ID of spark max controller 1
     * @param motor2DeviceID The CAN ID of spark max controller 2
     * @param leaderConfig   The config to be set on to the motor controller which
     *                       is the leader
     * @param followerConfig The config to be set on to the motor controller which
     *                       is the follower
     * @return motorPair a pair of motors with the first one as its leader and
     *         second one as the follower
     */
    public static List<CTRELite> buildKrakenPair(int motor1DeviceID, int motor2DeviceID, boolean invertFollower, Config leaderConfig, Config followerConfig, MotorConfig.Motor_Type motorType) {
        CTRELite krakenLeader = buildKraken(motor1DeviceID, leaderConfig, motorType);
        CTRELite krakenFollower = buildKraken(motor2DeviceID, leaderConfig, motorType);

        boolean isMotor1Error = krakenLeader.isErrored();
        boolean isMotor2Error = krakenFollower.isErrored();

        if (isMotor1Error && isMotor2Error) {
            MustangNotifications.reportError("krakenControllerID %s and krakenControllerID %s are broken", krakenLeader.getDeviceID(), krakenFollower.getDeviceID());
        } else if (isMotor2Error) {
            MustangNotifications.reportWarning("krakenControllerID %s is broken.", krakenFollower.getDeviceID());
        } else if (isMotor1Error) {
            MustangNotifications.reportWarning("krakenControllerID %s is broken. Switching to krakenControllerID %s", krakenLeader.getDeviceID(), krakenFollower.getDeviceID());
            CTRELite krakenTemp = krakenLeader;
            krakenLeader = krakenFollower;
            krakenFollower = krakenTemp;
        }
        // Tells the leader controller explicitly to not be following any other, to
        // avoid potential issues.
        // Refer to:
        // https://www.chiefdelphi.com/t/spark-max-follower-with-lower-can-id-than-leader-causes-4-stutters-sec-until-power-cycled/378716/12
        krakenFollower.setControl(new Follower(krakenLeader.getDeviceID(), invertFollower));
        List<CTRELite> motorPair = Arrays.asList(krakenLeader, krakenFollower);
        ConsoleLogger.consoleLog("krakenLeaderID %s, krakenFollowerID %s", krakenLeader.getDeviceID(), krakenFollower.getDeviceID());
        return motorPair;
    }

}