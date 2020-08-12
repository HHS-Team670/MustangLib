package frc.team670.mustanglib.utils.motorcontroller.check;

import edu.wpi.first.wpilibj.Timer;

import frc.team670.mustanglib.subsystems.MustangSubsystemBase;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

/**
 * For performing basic tests to check functionality of a subsystem's motors.
 * 
 * @author ctychen
 */
public abstract class MotorChecker<T> {

    public static class Config {
        // unscientifically-deterimined values, change them if needed
        public double minCurrent = 5;
        public double currentError = 3;

        public double minRPM = 1000;
        public double rpmError = 500;

        public DoubleSupplier rpmSupplier = null;

        public double runTime = 5;
        public double testOutputPercent = 0.5;
    }

    public static class MotorConfig<T> {
        public String name;
        public T motor;

        public MotorConfig(String name, T motor) {
            this.name = name;
            this.motor = motor;
        }
    }

    protected ArrayList<MotorConfig<T>> motorsToCheck;

    public abstract void storeConfig();

    public abstract void restoreConfig();

    public abstract void setOutput(T motor, double output);

    public abstract double getCurrent(T motor);

    /**
     * Runs basic tests for any motor, specifically, the ability to run at a
     * consistent current and RPM.
     * 
     * @param subsystem     Subsystem to run the motor check on
     * @param motorsToCheck
     * @param checkerConfig
     * @return true if all tests pass, false if any of the following happens: motor
     *         is running at below a defined minimum current or RPM, and/or current
     *         or RPM is varying too much.
     */
    public boolean check(MustangSubsystemBase subsystem, ArrayList<MotorConfig<T>> motorsToCheck,
            Config checkerConfig) {

        System.out.println("Checking MustangSubsystemBase " + subsystem.getClass() + " for " + motorsToCheck.size()
                + " motors...");

        boolean failed = false;

        ArrayList<Double> currents = new ArrayList<>();
        ArrayList<Double> rpms = new ArrayList<>();

        this.motorsToCheck = motorsToCheck;

        storeConfig();

        for (MotorConfig<T> config : motorsToCheck) {
            setOutput(config.motor, 0.0);
        }

        for (MotorConfig<T> config : motorsToCheck) {
            System.out.println("Checking: " + config.name);

            setOutput(config.motor, checkerConfig.testOutputPercent);
            Timer.delay(checkerConfig.runTime);

            double current = getCurrent(config.motor);
            currents.add(current);
            System.out.print("Current: " + current);

            double rpm = Double.NaN;
            if (checkerConfig.rpmSupplier != null) {
                rpm = checkerConfig.rpmSupplier.getAsDouble();
                rpms.add(rpm);
                System.out.print(" RPM: " + rpm + "\n");
            }

            setOutput(config.motor, 0.0);

            if (current < checkerConfig.minCurrent) {
                System.out.println("Current check failed for: " + config.name + ", the target current should be "
                        + checkerConfig.minCurrent + "!\n");
                failed = true;
            }
            if (checkerConfig.rpmSupplier != null) {
                if (rpm < checkerConfig.minRPM) {
                    System.out.println("RPM check failed for: " + config.name + ", the  target RPM should be "
                            + checkerConfig.minRPM + "!\n");
                    failed = true;
                }
            }

        }

        if (currents.size() > 0) {
            double average = currents.stream().mapToDouble(val -> val).average().getAsDouble();

            if (!closeTo(currents, average, checkerConfig.currentError)) {
                System.out.println("Check failed: Current is varying too much");
                failed = true;
            }
        }

        if (rpms.size() > 0) {
            double average = rpms.stream().mapToDouble(val -> val).average().getAsDouble();

            if (!closeTo(rpms, average, checkerConfig.rpmError)) {
                System.out.println("Check failed: RPM is varying too much");
                failed = true;
            }
        }

        restoreConfig();

        return !failed;
    }

    public static boolean closeTo(final List<Double> list, double value, double error) {
        boolean result = true;
        for (Double value_in : list) {
            result &= aboutEquals(value_in, value, error);
        }
        return result;
    }

    public static boolean aboutEquals(double a, double b, double error) {
        return (a - error <= b) && (a + error >= b);
    }
}