package frc.team670.mustanglib.swervelib.ctre;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.team670.mustanglib.swervelib.DriveController;
import frc.team670.mustanglib.swervelib.DriveControllerFactory;
import frc.team670.mustanglib.swervelib.ModuleConfiguration;

public final class KrakenX60DriveControllerFactoryBuilder {
    private static final double TICKS_PER_ROTATION = 2048.0;


    private double nominalVoltage = Double.NaN; 
    private double currentLimit = Double.NaN;

    public KrakenX60DriveControllerFactoryBuilder withVoltageCompensation(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public DriveControllerFactory<ControllerImplementation, Integer> build() {
        return new FactoryImplementation();
    }

    public KrakenX60DriveControllerFactoryBuilder withCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    private class FactoryImplementation
            implements DriveControllerFactory<ControllerImplementation, Integer> {
        @Override
        public ControllerImplementation create(Integer id, String canbus,
                ModuleConfiguration moduleConfiguration) {
            TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
            
            double sensorPositionCoefficient = Math.PI * moduleConfiguration.getWheelDiameter()
                    * moduleConfiguration.getDriveReduction() / TICKS_PER_ROTATION;
            double sensorVelocityCoefficient = sensorPositionCoefficient * 10.0;

            if (hasCurrentLimit()) {
                motorConfiguration.CurrentLimits.SupplyCurrentLimit = currentLimit; // TODO lines 54-59?
                motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
            }
            // motorConfiguration.CurrentLimits.SupplyCurrentLimit = 60;
            // motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
            // motorConfiguration.CurrentLimits.StatorCurrentLimit= 100;
            // motorConfiguration.CurrentLimits.StatorCurrentLimitEnable=true;
            // motorConfiguration.CurrentLimits.SupplyTimeThreshold=0.25;
            // motorConfiguration.CurrentLimits.SupplyCurrentThreshold=120;
            

            TalonFX motor = new TalonFX(id, canbus);
            CtreUtils.checkCtreError(motor.getConfigurator().apply(motorConfiguration),
                    "Failed to configure Kraken X60");
            

            motor.setNeutralMode(NeutralModeValue.Brake);

            motor.setInverted(moduleConfiguration.isDriveInverted()); // is inverted in clockwise or not? we don't know

            // Reduce CAN status frame rates
            double positionConversionFactor = Math.PI * moduleConfiguration.getWheelDiameter()
                    * moduleConfiguration.getDriveReduction();

            return new ControllerImplementation(motor, sensorVelocityCoefficient, positionConversionFactor);
        }
    }

    private class ControllerImplementation implements DriveController {
        private final TalonFX motor;
        private final double sensorVelocityCoefficient;
        private final double nominalVoltage = hasVoltageCompensation()
                ? KrakenX60DriveControllerFactoryBuilder.this.nominalVoltage
                : 12.0;
        private final double positionConversionFactor;

        private ControllerImplementation(TalonFX motor, double sensorVelocityCoefficient, double positionConversionFactor) {
            this.motor = motor;
            this.sensorVelocityCoefficient = sensorVelocityCoefficient;
            this.positionConversionFactor = positionConversionFactor;
        }

        @Override
        public Object getDriveMotor() {
            return this.motor;
        }

        @Override
        public void setReferenceVoltage(double voltage) {
            motor.setVoltage(voltage ); // not in percentage
        }

        @Override
        public double getStateVelocity() {
            return (motor.getRotorVelocity().getValue() * sensorVelocityCoefficient);
        }

        @Override
        public double getDistanceMoved() {
            var rotorPosSignal = motor.getRotorPosition();
            return rotorPosSignal.getValueAsDouble() * positionConversionFactor;
        }
    }
}
