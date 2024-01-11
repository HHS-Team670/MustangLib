package frc.team670.mustanglib.swervelib.ctre;

import com.ctre.phoenix.motorcontrol.*;
//import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
            
            double sensorPositionCoefficient = Math.PI * moduleConfiguration.getWheelDiameter()
                    * moduleConfiguration.getDriveReduction() / TICKS_PER_ROTATION;
            double sensorVelocityCoefficient = sensorPositionCoefficient * 10.0;

            if (hasCurrentLimit()) {
                motorConfiguration.CurrentLimits.SupplyCurrentLimit = currentLimit;
                motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
            }

            TalonFX motor = new TalonFX(id, canbus);
            CtreUtils.checkCtreError(motor.configAllSettings(motorConfiguration),
                    "Failed to configure Falcon 500");

            motor.setNeutralMode(NeutralModeValue.Brake);

            motor.setInverted(moduleConfiguration.isDriveInverted() ? TalonFXInvertType.Clockwise
                    : TalonFXInvertType.CounterClockwise);
            motor.setSensorPhase(true);

            // Reduce CAN status frame rates

            return new ControllerImplementation(motor, sensorVelocityCoefficient);
        }
    }

    private class ControllerImplementation implements DriveController {
        private final WPI_TalonFX motor;
        private final double sensorVelocityCoefficient;
        private final double nominalVoltage = hasVoltageCompensation()
                ? KrakenX60DriveControllerFactoryBuilder.this.nominalVoltage
                : 12.0;

        private ControllerImplementation(WPI_TalonFX motor, double sensorVelocityCoefficient) {
            this.motor = motor;
            this.sensorVelocityCoefficient = sensorVelocityCoefficient;
        }

        @Override
        public Object getDriveMotor() {
            return this.motor;
        }

        @Override
        public void setReferenceVoltage(double voltage) {
            motor.set(TalonFXControlMode.PercentOutput, voltage / nominalVoltage);
        }

        @Override
        public double getStateVelocity() {
            return motor.getSelectedSensorVelocity() * sensorVelocityCoefficient;
        }

        @Override
        public double getDistanceMoved() {
            throw new UnsupportedOperationException();
            // return motor.getSelectedSensorPosition();
        }
    }
}
