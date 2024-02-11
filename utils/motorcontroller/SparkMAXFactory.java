package frc.team670.mustanglib.utils.motorcontroller;

import java.util.Arrays;
import java.util.List;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.REVLibError;

import frc.team670.mustanglib.utils.ConsoleLogger;
import frc.team670.mustanglib.utils.MustangNotifications;

/**
 * Utility class for configuring a SparkMAX to default settings and resetting to
 * factory defaults.
 * 
 * @author ctychen, ruchidixit
 */
public class SparkMAXFactory {

    public static class Config {

        public boolean BURN_FACTORY_DEFAULT_FLASH = false;
        public IdleMode DEFAULT_MODE = IdleMode.kCoast;
        public boolean INVERTED = false;

        public int STATUS_FRAME_0_RATE_MS = 10; // Default status frame periods from https://docs.revrobotics.com/brushless/spark-max/control-interfaces#periodic-status-frames
        public int STATUS_FRAME_1_RATE_MS = 20;
        public int STATUS_FRAME_2_RATE_MS = 20;
        public int STATUS_FRAME_3_RATE_MS = 50;
        public int STATUS_FRAME_4_RATE_MS = 20;
        public int STATUS_FRAME_5_RATE_MS = 200;
        public int STATUS_FRAME_6_RATE_MS = 200;

        public double OPEN_LOOP_RAMP_RATE = 0.0;
        public double CLOSED_LOOP_RAMP_RATE = 0.0;

        public boolean ENABLE_VOLTAGE_COMPENSATION = false;
        public double NOMINAL_VOLTAGE = 12.0;

    }

    public static final Config defaultConfig = new Config(); // For motors where we care about position and velocity
    public static final Config defaultVelocityConfig = new Config();// For motors were we only care about precise velocity and not precise position 
    public static final Config defaultPositionConfig = new Config(); // For motors were we only care about precise position and not precise velocity 
    public static final Config defaultLowUpdateRateConfig = new Config(); // For motors where we neither care about precise velocity or position
    public static final Config defaultFollowerConfig = new Config(); //For follower motors

    //We leave frames 0 and 1 at default for non follower motors because we often need to track motor applied output (Frame 0) and current (Frame 1) regadless of if we need precise position or velocity
    static {
        defaultFollowerConfig.STATUS_FRAME_0_RATE_MS = 1000;
        defaultFollowerConfig.STATUS_FRAME_1_RATE_MS = 1000;
        defaultFollowerConfig.STATUS_FRAME_2_RATE_MS = 1000;
        defaultFollowerConfig.STATUS_FRAME_3_RATE_MS = 1000;
        defaultFollowerConfig.STATUS_FRAME_4_RATE_MS = 1000;
        defaultFollowerConfig.STATUS_FRAME_5_RATE_MS = 1000;
        defaultFollowerConfig.STATUS_FRAME_6_RATE_MS = 1000;

        defaultVelocityConfig.STATUS_FRAME_2_RATE_MS = 1000;
        defaultVelocityConfig.STATUS_FRAME_3_RATE_MS = 1000;
        defaultVelocityConfig.STATUS_FRAME_4_RATE_MS = 1000;
        defaultVelocityConfig.STATUS_FRAME_5_RATE_MS = 1000;
        defaultVelocityConfig.STATUS_FRAME_6_RATE_MS = 1000;

        defaultPositionConfig.STATUS_FRAME_3_RATE_MS = 1000;
        defaultPositionConfig.STATUS_FRAME_4_RATE_MS = 1000;
        defaultPositionConfig.STATUS_FRAME_5_RATE_MS = 1000;
        defaultPositionConfig.STATUS_FRAME_6_RATE_MS = 1000;

        defaultLowUpdateRateConfig.STATUS_FRAME_2_RATE_MS = 1000;
        defaultLowUpdateRateConfig.STATUS_FRAME_3_RATE_MS = 1000;
        defaultLowUpdateRateConfig.STATUS_FRAME_4_RATE_MS = 1000;
        defaultLowUpdateRateConfig.STATUS_FRAME_5_RATE_MS = 1000;
        defaultLowUpdateRateConfig.STATUS_FRAME_6_RATE_MS = 1000;
    }

    /**
     * Creates a SparkMAXLite with factory settings.
     */
    public static SparkMAXLite buildFactorySparkMAX(int deviceID, MotorConfig.Motor_Type motorType) {
        return buildSparkMAX(deviceID, defaultConfig, motorType);
    }



    public static SparkMAXLite setPermanentFollower(int deviceID, SparkMAXLite leader) {
        return setPermanentFollower(deviceID, leader, false);
    }

    public static SparkMAXLite setPermanentFollower(int deviceID, SparkMAXLite leader, boolean inverted) {
        SparkMAXLite sparkMax = buildSparkMAX(deviceID, defaultFollowerConfig, leader.getMotor());
        sparkMax.follow(leader, inverted);
        return sparkMax;
    }

    /**
     * 
     * @param deviceID  CAN ID of this SparkMax
     * @param config    The configuration to set this for, ex. default or
     *                  defaultFollower
     * @param motorType The kind of motor this controller will be using
     * @return SparkMAXLite set to this configuration with current limit
     */
    public static SparkMAXLite buildSparkMAX(int deviceID, Config config, MotorConfig.Motor_Type motorType) {
        SparkMAXLite sparkMax = new SparkMAXLite(deviceID, motorType);
        sparkMax.restoreFactoryDefaults();
        sparkMax.set(ControlType.kDutyCycle, 0);
        sparkMax.setInverted(config.INVERTED);
        sparkMax.setSmartCurrentLimit(MotorConfig.MOTOR_MAX_CURRENT.get(motorType));
        sparkMax.enableVoltageCompensation(12);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, config.STATUS_FRAME_0_RATE_MS);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, config.STATUS_FRAME_1_RATE_MS);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, config.STATUS_FRAME_2_RATE_MS);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, config.STATUS_FRAME_3_RATE_MS);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, config.STATUS_FRAME_4_RATE_MS);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, config.STATUS_FRAME_5_RATE_MS);
        sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, config.STATUS_FRAME_6_RATE_MS);

        return sparkMax;
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
    public static List<SparkMAXLite> buildFactorySparkMAXPair(int motor1DeviceID, int motor2DeviceID, boolean invertFollower, MotorConfig.Motor_Type motorType) {
        return buildSparkMAXPair(motor1DeviceID, motor2DeviceID, invertFollower, defaultConfig, defaultFollowerConfig, motorType);
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
    public static List<SparkMAXLite> buildSparkMAXPair(int motor1DeviceID, int motor2DeviceID, boolean invertFollower, Config config, MotorConfig.Motor_Type motorType) {
        return buildSparkMAXPair(motor1DeviceID, motor2DeviceID, invertFollower, config, config, motorType);
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
    public static List<SparkMAXLite> buildSparkMAXPair(int motor1DeviceID, int motor2DeviceID, boolean invertFollower, Config leaderConfig, Config followerConfig, MotorConfig.Motor_Type motorType) {
        SparkMAXLite sparkMaxLeader = buildSparkMAX(motor1DeviceID, leaderConfig, motorType);
        SparkMAXLite sparkMaxFollower = buildSparkMAX(motor2DeviceID, leaderConfig, motorType);

        REVLibError sparkMaxLeaderError = sparkMaxLeader.getLastError();
        REVLibError sparkMaxFollowerError = sparkMaxFollower.getLastError();

        boolean isMotor1Error = sparkMaxLeaderError != REVLibError.kOk && sparkMaxLeaderError != null;
        boolean isMotor2Error = sparkMaxFollowerError != REVLibError.kOk && sparkMaxFollowerError != null;

        if (isMotor1Error && isMotor2Error) {
            MustangNotifications.reportError("SparkMaxControllerID %s and SparkMaxControllerID %s are broken", sparkMaxLeader.getDeviceId(), sparkMaxFollower.getDeviceId());
        } else if (isMotor2Error) {
            MustangNotifications.reportWarning("SparkMaxControllerID %s is broken.", sparkMaxFollower.getDeviceId());
        } else if (isMotor1Error) {
            MustangNotifications.reportWarning("SparkMaxControllerID %s is broken. Switching to SparkMaxControllerID %s", sparkMaxLeader.getDeviceId(), sparkMaxFollower.getDeviceId());
            SparkMAXLite sparkMaxTemp = sparkMaxLeader;
            sparkMaxLeader = sparkMaxFollower;
            sparkMaxFollower = sparkMaxTemp;
        }
        // Tells the leader controller explicitly to not be following any other, to
        // avoid potential issues.
        // Refer to:
        // https://www.chiefdelphi.com/t/spark-max-follower-with-lower-can-id-than-leader-causes-4-stutters-sec-until-power-cycled/378716/12
        sparkMaxLeader.follow(ExternalFollower.kFollowerDisabled, 0);
        sparkMaxFollower.follow(sparkMaxLeader, invertFollower);
        List<SparkMAXLite> motorPair = Arrays.asList(sparkMaxLeader, sparkMaxFollower);
        ConsoleLogger.consoleLog("SparkMaxLeaderID %s, SparkMaxFollowerID %s", sparkMaxLeader.getDeviceId(), sparkMaxFollower.getDeviceId());
        return motorPair;
    }

}