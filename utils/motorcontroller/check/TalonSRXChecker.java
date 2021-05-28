package frc.team670.mustanglib.utils.motorcontroller.check;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.TalonSRXLite;

import java.util.ArrayList;

/**
 * Basic test for motors using TalonSRX controllers
 * 
 * @author ctychen
 */
public class TalonSRXChecker extends MotorChecker<TalonSRX> {

    private static class StoredTalonSRXConfig {
        public ControlMode mode;
        public double setValue;
    }

    public ArrayList<StoredTalonSRXConfig> mStoredConfigs = new ArrayList<>();

    @Override
    public void storeConfig() {
        // record previous configuration for all talons
        for (MotorConfig<TalonSRX> config : motorsToCheck) {
            TalonSRXLite talon = (TalonSRXLite) config.motor;

            StoredTalonSRXConfig configuration = new StoredTalonSRXConfig();
            configuration.mode = talon.getControlMode();
            configuration.setValue = talon.getLastSet();

            mStoredConfigs.add(configuration);
        }
    }

    @Override
    public void restoreConfig() {
        for (int i = 0; i < motorsToCheck.size(); ++i) {
            motorsToCheck.get(i).motor.set(mStoredConfigs.get(i).mode, mStoredConfigs.get(i).setValue);
        }
    }

    @Override
    public void setOutput(TalonSRX motor, double output) {
        motor.set(ControlMode.PercentOutput, output);
    }

    @Override
    public double getCurrent(TalonSRX motor) {
        return motor.getStatorCurrent();
    }

    public static boolean checkMotors(MustangSubsystemBase subsystem, ArrayList<MotorConfig<TalonSRX>> motorsToCheck,
            Config checkerConfig) {
        TalonSRXChecker checker = new TalonSRXChecker();
        return checker.checkMotors(subsystem, motorsToCheck, checkerConfig);
    }

}