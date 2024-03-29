package frc.team670.mustanglib.swervelib.rev;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.team670.mustanglib.swervelib.DriveController;
import frc.team670.mustanglib.swervelib.DriveControllerFactory;
import frc.team670.mustanglib.swervelib.ModuleConfiguration;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;

import static frc.team670.mustanglib.swervelib.rev.RevUtils.checkNeoError;

public final class NeoDriveControllerFactoryBuilder {
    private double nominalVoltage = Double.NaN;
    private double currentLimit = Double.NaN;

    public NeoDriveControllerFactoryBuilder withVoltageCompensation(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public NeoDriveControllerFactoryBuilder withCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    public DriveControllerFactory<ControllerImplementation, Integer> build() {
        return new FactoryImplementation();
    }

    private class FactoryImplementation
            implements DriveControllerFactory<ControllerImplementation, Integer> {
        @Override
        public ControllerImplementation create(Integer id, String _canbus,
                ModuleConfiguration moduleConfiguration) {
            CANSparkMax motor = new CANSparkMax(id, CANSparkLowLevel.MotorType.kBrushless);
            motor.setInverted(moduleConfiguration.isDriveInverted());

            // Setup voltage compensation
            if (hasVoltageCompensation()) {
                checkNeoError(motor.enableVoltageCompensation(nominalVoltage),
                        "Failed to enable voltage compensation");
            }

            if (hasCurrentLimit()) {
                checkNeoError(motor.setSmartCurrentLimit((int) currentLimit),
                        "Failed to set current limit for NEO");
            }

            checkNeoError(
                    motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus0, 100),
                    "Failed to set periodic status frame 0 rate");
            checkNeoError(
                    motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20),
                    "Failed to set periodic status frame 1 rate");
            checkNeoError(
                    motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20),
                    "Failed to set periodic status frame 2 rate");
            checkNeoError(
                    motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3,  SparkMAXFactory.defaultLowUpdateRateConfig.STATUS_FRAME_3_RATE_MS),
                    "Failed to set periodic status frame 3 rate");
            checkNeoError(
                    motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus4,  SparkMAXFactory.defaultLowUpdateRateConfig.STATUS_FRAME_4_RATE_MS),
                    "Failed to set periodic status frame 4 rate");
            checkNeoError(
                    motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus5,  SparkMAXFactory.defaultLowUpdateRateConfig.STATUS_FRAME_5_RATE_MS),
                    "Failed to set periodic status frame 5 rate");
            checkNeoError(
                    motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus6,  SparkMAXFactory.defaultLowUpdateRateConfig.STATUS_FRAME_6_RATE_MS),
                    "Failed to set periodic status frame 6 rate");
            // Set neutral mode to brake
            motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

            // Setup encoder
            RelativeEncoder encoder = motor.getEncoder();
            double positionConversionFactor = Math.PI * moduleConfiguration.getWheelDiameter()
                    * moduleConfiguration.getDriveReduction();
            encoder.setPositionConversionFactor(positionConversionFactor);
            encoder.setVelocityConversionFactor(positionConversionFactor / 60.0);

            return new ControllerImplementation(motor, encoder);
        }
    }

    private static class ControllerImplementation implements DriveController {
        private final CANSparkMax motor;
        private final RelativeEncoder encoder;

        private ControllerImplementation(CANSparkMax motor, RelativeEncoder encoder) {
            this.motor = motor;
            this.encoder = encoder;
        }

        @Override
        public Object getDriveMotor() {
            return this.motor;
        }

        @Override
        public void setReferenceVoltage(double voltage) {
            motor.setVoltage(voltage);
        }

        @Override
        public double getStateVelocity() {
            return encoder.getVelocity();
        }

        @Override
        public double getDistanceMoved() {
            return encoder.getPosition();
        }
    }
}
