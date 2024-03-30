package frc.team670.mustanglib.utils.motorcontroller;

import java.util.Arrays;
import java.util.List;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.ExternalFollower;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.REVLibError;

import frc.team670.mustanglib.swervelib.ctre.CtreUtils;
import frc.team670.mustanglib.utils.ConsoleLogger;
import frc.team670.mustanglib.utils.MustangNotifications;

public class TalonFXFactory {
        public static class Config {

        public NeutralModeValue DEFAULT_MODE = NeutralModeValue.Coast;
        public boolean INVERTED = false;
        public int CURRENT_LIMIT = 40;
        public double SUPPLY_TIME_THRESHOLD=0.25;
        public double SUPPLY_CURRENT_THRESHOLD=120;
        public double FAULT_UPDATE_FREQUENCY=10;
        public double POSITION_UPDATE_FREQUENCY=50;
        public double VELOCITY_UPDATE_FREQUENCY=50;
        public double ROTOR_POSITION_UPDATE_FREQUENCY=50;
        public double CURRENT_UPDATE_FREQUENCY=50;
        public double VOLTAGE_UPDATE_FREQUENCY=50;


   

        public static TalonFXFactory.Config copy(TalonFXFactory.Config config){
            Config copy = new Config();
            copy.DEFAULT_MODE = config.DEFAULT_MODE;
            copy.INVERTED = config.INVERTED;
            copy.CURRENT_LIMIT = config.CURRENT_LIMIT;

            return copy;            

        }

    }
    public static final Config defaultConfig = new Config(); // For motors where we care about position and velocity
    public static final Config defaultFollowerConfig = new Config(); // For

    /**
     * Creates a TalonFX with factory settings.
     */
    public static TalonFX buildFactoryTalonFX(int deviceID, MotorConfig.Motor_Type motorType) {
        return buildTalonFX(deviceID, defaultConfig, motorType);
    }



    public static TalonFX setPermanentFollower(int deviceID, TalonFX leader, MotorConfig.Motor_Type motorType) {
        return setPermanentFollower(deviceID, leader, false,motorType);
    }

    public static TalonFX setPermanentFollower(int deviceID, TalonFX leader, boolean inverted,MotorConfig.Motor_Type motorType) {
        TalonFX talonfx = buildTalonFX(deviceID, defaultFollowerConfig,motorType);
        talonfx.setControl(new Follower(leader.getDeviceID(), inverted));
        return talonfx;
    }

    /**
     * 
     * @param deviceID  CAN ID of this SparkMax
     * @param config    The configuration to set this for, ex. default or
     *                  defaultFollower
     * @param motorType The kind of motor this controller will be using
     * @return TalonFX set to this configuration with current limit
     */
    public static TalonFX buildTalonFX(int deviceID, Config config, MotorConfig.Motor_Type motorType) {
        TalonFX talonfx = new TalonFX(deviceID);
        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        motorConfiguration.CurrentLimits.SupplyCurrentLimit = config.CURRENT_LIMIT;
        motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfiguration.CurrentLimits.SupplyTimeThreshold=config.SUPPLY_TIME_THRESHOLD;
        motorConfiguration.CurrentLimits.SupplyCurrentThreshold=config.SUPPLY_CURRENT_THRESHOLD;
        BaseStatusSignal.setUpdateFrequencyForAll(config.FAULT_UPDATE_FREQUENCY, talonfx.getStickyFaultField()); //period 0
        BaseStatusSignal.setUpdateFrequencyForAll(config.POSITION_UPDATE_FREQUENCY, talonfx.getPosition()); //period 0
        BaseStatusSignal.setUpdateFrequencyForAll(config.VELOCITY_UPDATE_FREQUENCY, talonfx.getVelocity()); //period 0
        BaseStatusSignal.setUpdateFrequencyForAll(config.ROTOR_POSITION_UPDATE_FREQUENCY, talonfx.getRotorPosition()); //period 0
        BaseStatusSignal.setUpdateFrequencyForAll(config.CURRENT_UPDATE_FREQUENCY, talonfx.getSupplyCurrent(),talonfx.getStatorCurrent()); //period 0
        BaseStatusSignal.setUpdateFrequencyForAll(config.VOLTAGE_UPDATE_FREQUENCY, talonfx.getMotorVoltage(),talonfx.getSupplyVoltage()); //period 0


    
        // Optimize bus utilization
        talonfx.optimizeBusUtilization(1.0);
        CtreUtils.checkCtreError(talonfx.getConfigurator().apply(motorConfiguration),
                    "Failed to configure Kraken X60");
        

        talonfx.setInverted(config.INVERTED);
        talonfx.setNeutralMode(config.DEFAULT_MODE);

      
    
        
        return talonfx;
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
    public static List<TalonFX> buildFactoryTalonFXPair(int motor1DeviceID, int motor2DeviceID, boolean invertFollower, MotorConfig.Motor_Type motorType) {
        return buildTalonFXPair(motor1DeviceID, motor2DeviceID, invertFollower, defaultConfig, defaultFollowerConfig, motorType);
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
    public static List<TalonFX> buildTalonFXPair(int motor1DeviceID, int motor2DeviceID, boolean invertFollower, Config config, MotorConfig.Motor_Type motorType) {
        return buildTalonFXPair(motor1DeviceID, motor2DeviceID, invertFollower, config, config, motorType);
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
    public static List<TalonFX> buildTalonFXPair(int motor1DeviceID, int motor2DeviceID, boolean invertFollower, Config leaderConfig, Config followerConfig, MotorConfig.Motor_Type motorType) {
        TalonFX talonfxLeader = buildTalonFX(motor1DeviceID, leaderConfig, motorType);
        TalonFX talonfxFollower = buildTalonFX(motor2DeviceID, leaderConfig, motorType);

        int talonfxLeaderError = talonfxLeader.getFaultField().getValue();
        int talonfxFollowerError = talonfxFollower.getFaultField().getValue();

        boolean isMotor1Error = talonfxLeaderError==0;
        boolean isMotor2Error = talonfxFollowerError==0;

        if (isMotor1Error && isMotor2Error) {
            MustangNotifications.reportError("TalonFXID %s and TalonFXID %s are broken", talonfxLeader.getDeviceID(), talonfxFollower.getDeviceID());
        } else if (isMotor2Error) {
            MustangNotifications.reportWarning("TalonFXID %s is broken.", talonfxFollower.getDeviceID());
        } else if (isMotor1Error) {
            MustangNotifications.reportWarning("TalonFXID %s is broken. Switching to SparkMaxControllerID %s", talonfxLeader.getDeviceID(), talonfxFollower.getDeviceID());
            TalonFX talonfxTemp = talonfxLeader;
            talonfxLeader = talonfxFollower;
            talonfxFollower = talonfxTemp;
        }
        // Tells the leader controller explicitly to not be following any other, to
        // avoid potential issues.
        // Refer to:
        // https://www.chiefdelphi.com/t/spark-max-follower-with-lower-can-id-than-leader-causes-4-stutters-sec-until-power-cycled/378716/12
        talonfxFollower.setControl(new Follower(talonfxLeader.getDeviceID(), invertFollower));
        List<TalonFX> motorPair = Arrays.asList(talonfxLeader, talonfxFollower);
        ConsoleLogger.consoleLog("SparkMaxLeaderID %s, SparkMaxFollowerID %s", talonfxLeader.getDeviceID(), talonfxFollower.getDeviceID());
        return motorPair;
    }


}
