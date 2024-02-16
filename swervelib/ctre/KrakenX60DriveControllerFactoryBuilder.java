package frc.team670.mustanglib.swervelib.ctre;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;

import frc.team670.mustanglib.swervelib.DriveController;
import frc.team670.mustanglib.swervelib.DriveControllerFactory;
import frc.team670.mustanglib.swervelib.ModuleConfiguration;

public final class KrakenX60DriveControllerFactoryBuilder {
    // REPLACE CONSTANTS, THEY ARE INVALID
    private static final double TICKS_PER_ROTATION = 2048.0;

    private static final int CAN_TIMEOUT_MS = 250; 
    private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;

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
            
            double encoderDistancePerPulse = (moduleConfiguration.getWheelDiameter() * Math.PI) / TICKS_PER_ROTATION;
            double sensorPositionCoefficient = Math.PI * moduleConfiguration.getWheelDiameter()
                    * moduleConfiguration.getDriveReduction() / TICKS_PER_ROTATION;
            double sensorVelocityCoefficient = sensorPositionCoefficient * 10.0;

            if (hasCurrentLimit()) {
                motorConfiguration.CurrentLimits.SupplyCurrentLimit = currentLimit;
                motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
            }

            TalonFX motor = new TalonFX(id, canbus);
            CtreUtils.checkCtreError(motor.getConfigurator().apply(motorConfiguration),
                    "Failed to configure Kraken X60");

            motor.setNeutralMode(NeutralModeValue.Brake);

            motor.setInverted(moduleConfiguration.isDriveInverted()); // is inverted in clockwise or not? we don't know

            // Reduce CAN status frame rates
            double positionConversionFactor = Math.PI * moduleConfiguration.getWheelDiameter()
                    * moduleConfiguration.getDriveReduction();

            return new ControllerImplementation(motor, sensorVelocityCoefficient, encoderDistancePerPulse);
        }
    }

    private class ControllerImplementation implements DriveController {
        private final TalonFX motor;
        private final double sensorVelocityCoefficient;
        private final double nominalVoltage = hasVoltageCompensation()
                ? KrakenX60DriveControllerFactoryBuilder.this.nominalVoltage
                : 12.0;
        private final double encoderDistancePerPulse;

        private ControllerImplementation(TalonFX motor, double sensorVelocityCoefficient, double encoderDistancePerPulse) {
            this.motor = motor;
            this.sensorVelocityCoefficient = sensorVelocityCoefficient;
            this.encoderDistancePerPulse = encoderDistancePerPulse;
        }

        @Override
        public Object getDriveMotor() {
            return this.motor;
        }

        @Override
        public void setReferenceVoltage(double voltage) {
            motor.setVoltage(voltage / nominalVoltage); // not in percentage
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
