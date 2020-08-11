package frc.team670.robot.utils.motorcontroller.check;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.utils.motorcontroller.SparkMAXLite;

import java.util.ArrayList;

/**
 * Basic test for motors using SparkMAX controllers
 * 
 * @author ctychen
 */
public class SparkMAXChecker extends MotorChecker<CANSparkMax> {

    public static class StoredSparkConfig {
        CANSparkMax leader = null;
    }

    public ArrayList<StoredSparkConfig> mStoredConfigs = new ArrayList<>();

    @Override
    public void storeConfig() {
        // record the last configs used on these motors
        for (MotorConfig<CANSparkMax> config : motorsToCheck) {
            SparkMAXLite spark = (SparkMAXLite) config.motor;

            StoredSparkConfig configuration = new StoredSparkConfig();
            configuration.leader = spark.getLeader();

            mStoredConfigs.add(configuration);
            spark.restoreFactoryDefaults();
        }
    }

    @Override
    public void restoreConfig() {
        for (int i = 0; i < motorsToCheck.size(); ++i) {
            if (mStoredConfigs.get(i).leader != null) {
                motorsToCheck.get(i).motor.follow(mStoredConfigs.get(i).leader);
            }
        }
    }

    @Override
    public void setOutput(CANSparkMax motor, double output) {
        motor.getPIDController().setReference(output, ControlType.kDutyCycle);
    }

    @Override
    public double getCurrent(CANSparkMax motor) {
        return motor.getOutputCurrent();
    }

    public static boolean checkMotors(MustangSubsystemBase subsystem, ArrayList<MotorConfig<CANSparkMax>> motorsToCheck,
            Config checkerConfig) {
        SparkMAXChecker checker = new SparkMAXChecker();
        return checker.checkMotors(subsystem, motorsToCheck, checkerConfig);
    }

}