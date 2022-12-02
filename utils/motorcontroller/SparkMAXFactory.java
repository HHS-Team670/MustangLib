package frc.team670.mustanglib.utils.motorcontroller;

import java.util.Arrays;
import java.util.List;

import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMax.IdleMode;

import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.MustangNotifications;
import frc.team670.mustanglib.utils.motorcontroller.defaultconfig.SparkMAXDefaults;

import com.revrobotics.CANSparkMax.ControlType;

/**
 * Utility class for configuring a SparkMAX to default settings and resetting to
 * factory defaults.
 * 
 * @author ctychen, ruchidixit, ajs256
 */
public class SparkMAXFactory {

    public static class Config {

        public boolean BURN_FACTORY_DEFAULT_FLASH = false;
        public IdleMode DEFAULT_MODE = IdleMode.kCoast;
        public boolean INVERTED = false;

        // sane defaults in case we don't set
        public int STATUS_FRAME_0_RATE_MS = 20;
        public int STATUS_FRAME_1_RATE_MS = 1000;
        public int STATUS_FRAME_2_RATE_MS = 1000;
        public int STATUS_FRAME_3_RATE_MS = 5000;
        public int STATUS_FRAME_4_RATE_MS = 5000;

        public double OPEN_LOOP_RAMP_RATE = 0.0;
        public double CLOSED_LOOP_RAMP_RATE = 0.0;

        public boolean ENABLE_VOLTAGE_COMPENSATION = false;
        public double NOMINAL_VOLTAGE = 12.0;

    }

    public static final Config defaultConfig = new Config();
    public static final Config defaultFollowerConfig = SparkMAXDefaults.UNCONTROLLED_NEO;

    /**
     * Creates a SparkMAXLite with factory settings.
     */
    public static SparkMAXLite buildFactorySparkMAX(int deviceID, MotorConfig.Motor_Type motorType) {
        return buildSparkMAX(deviceID, defaultConfig, motorType);
    }

    public static SparkMAXLite setPermanentFollower(int deviceID, SparkMAXLite leader) {
        SparkMAXLite sparkMax = buildSparkMAX(deviceID, defaultFollowerConfig, leader.getMotor());
        sparkMax.follow(leader);
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
        // TODO set status frames
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
    public static List<SparkMAXLite> buildFactorySparkMAXPair(int motor1DeviceID, int motor2DeviceID,
            boolean invertFollower, MotorConfig.Motor_Type motorType) {
        return buildSparkMAXPair(motor1DeviceID, motor2DeviceID, invertFollower, defaultConfig, defaultConfig, motorType);
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
    public static List<SparkMAXLite> buildSparkMAXPair(int motor1DeviceID, int motor2DeviceID, boolean invertFollower, Config config,
            MotorConfig.Motor_Type motorType) {
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
    public static List<SparkMAXLite> buildSparkMAXPair(int motor1DeviceID, int motor2DeviceID, boolean invertFollower, Config leaderConfig,
            Config followerConfig,MotorConfig.Motor_Type motorType) {
        SparkMAXLite sparkMaxLeader = buildSparkMAX(motor1DeviceID, leaderConfig, motorType);
        SparkMAXLite sparkMaxFollower = buildSparkMAX(motor2DeviceID, leaderConfig, motorType);

        REVLibError sparkMaxLeaderError = sparkMaxLeader.getLastError();
        REVLibError sparkMaxFollowerError = sparkMaxFollower.getLastError();

        boolean isMotor1Error = sparkMaxLeaderError != REVLibError.kOk && sparkMaxLeaderError != null;
        boolean isMotor2Error = sparkMaxFollowerError != REVLibError.kOk && sparkMaxFollowerError != null;

        if (isMotor1Error && isMotor2Error) {
            MustangNotifications.reportError("SparkMaxControllerID %s and SparkMaxControllerID %s are broken",
                    sparkMaxLeader.getDeviceId(), sparkMaxFollower.getDeviceId());
        } else if (isMotor2Error) {
            MustangNotifications.reportWarning("SparkMaxControllerID %s is broken.", sparkMaxFollower.getDeviceId());
        } else if (isMotor1Error) {
            MustangNotifications.reportWarning("SparkMaxControllerID %s is broken. Switching to SparkMaxControllerID %s",
                    sparkMaxLeader.getDeviceId(), sparkMaxFollower.getDeviceId());
            SparkMAXLite sparkMaxTemp = sparkMaxLeader;
            sparkMaxLeader = sparkMaxFollower;
            sparkMaxFollower = sparkMaxTemp;
        }
        // Tells the leader controller explicitly to not be following any other, to avoid potential issues.
        // Refer to: https://www.chiefdelphi.com/t/spark-max-follower-with-lower-can-id-than-leader-causes-4-stutters-sec-until-power-cycled/378716/12
        sparkMaxLeader.follow(ExternalFollower.kFollowerDisabled, 0);
        sparkMaxFollower.follow(sparkMaxLeader, invertFollower);
        List<SparkMAXLite> motorPair = Arrays.asList(sparkMaxLeader, sparkMaxFollower);
        Logger.consoleLog("SparkMaxLeaderID %s, SparkMaxFollowerID %s", sparkMaxLeader.getDeviceId(),
                sparkMaxFollower.getDeviceId());
        return motorPair;
    }

}