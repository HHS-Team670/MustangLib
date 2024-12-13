package frc.team670.mustanglib.swervelib.ctre;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.team670.mustanglib.swervelib.AbsoluteEncoder;
import frc.team670.mustanglib.swervelib.AbsoluteEncoderFactory;
import frc.team670.mustanglib.swervelib.ModuleConfiguration;
import frc.team670.mustanglib.swervelib.SteerConfiguration;
import frc.team670.mustanglib.swervelib.SteerController;
import frc.team670.mustanglib.swervelib.SteerControllerFactory;

public final class KrakenX44SteerControllerFactoryBuilder {
    private static final double TICKS_PER_ROTATION = 2048.0;
    //PID Configuration
    private double pidProportional = Double.NaN;
    private double pidIntegral = Double.NaN;
    private double pidDerivative = Double.NaN;
    private Slot0Configs talonSlot0Configs;

    private double nominalVoltage = Double.NaN; 
    private double currentLimit = Double.NaN;

    public KrakenX44SteerControllerFactoryBuilder withPidConstants(double proportional, double integral, double derivative) {
        this.pidProportional = proportional;
        this.pidIntegral = integral;
        this.pidDerivative = derivative;
        return this;
    }

    public boolean hasPidConstants() {
        return Double.isFinite(pidProportional) && Double.isFinite(pidIntegral) && Double.isFinite(pidDerivative);
    }

    public KrakenX44SteerControllerFactoryBuilder withVoltageCompensation(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public KrakenX44SteerControllerFactoryBuilder withCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }
    
    
    public <T> SteerControllerFactory<ControllerImplementation, SteerConfiguration<T>> build(
            AbsoluteEncoderFactory<T> encoderFactory) {
        return new FactoryImplementation<>(encoderFactory);
    }

public class FactoryImplementation<T>
            implements SteerControllerFactory<ControllerImplementation, SteerConfiguration<T>> {
        private final AbsoluteEncoderFactory<T> encoderFactory;

        public FactoryImplementation(AbsoluteEncoderFactory<T> encoderFactory) {
            this.encoderFactory = encoderFactory;
        }


        @Override
        public ControllerImplementation create(SteerConfiguration<T> steerConfiguration, String _canbus,
                ModuleConfiguration moduleConfiguration) {

            TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
            AbsoluteEncoder absoluteEncoder = encoderFactory.create(steerConfiguration.getEncoderConfiguration());

            double sensorPositionCoefficient = 2.0 * Math.PI * moduleConfiguration.getSteerReduction();
            double sensorVelocityCoefficient = 2.0 * Math.PI * moduleConfiguration.getSteerReduction() / 60.0;
            
            // if (hasCurrentLimit()) {
            //     motorConfiguration.CurrentLimits.SupplyCurrentLimit = currentLimit; // TODO lines 54-59?
            //     motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
            // }
           // These lines of code are configuring the current limits for the TalonFX motor controller.
           // Here's a breakdown of what each line is doing:
            motorConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
            motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
            motorConfiguration.CurrentLimits.StatorCurrentLimit= 100;
            motorConfiguration.CurrentLimits.StatorCurrentLimitEnable=true;
            motorConfiguration.CurrentLimits.SupplyTimeThreshold=0.25;
            motorConfiguration.CurrentLimits.SupplyCurrentThreshold=80;
            motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;


            TalonFX motor = new TalonFX(steerConfiguration.getMotorPort(), _canbus);
            CtreUtils.checkCtreError(motor.getConfigurator().apply(motorConfiguration),
                    "Failed to configure Kraken X44");
            
            talonSlot0Configs = new Slot0Configs();
            talonSlot0Configs.kP = pidProportional;
            talonSlot0Configs.kI = pidIntegral;
            talonSlot0Configs.kD = pidDerivative;
            CtreUtils.checkCtreError(motor.getConfigurator().apply(talonSlot0Configs),
                    "Failed to configure Kraken X44 PIDs");

            motor.setNeutralMode(NeutralModeValue.Brake);

            motor.setInverted(moduleConfiguration.isDriveInverted()); // is inverted in clockwise or not? we don't know

            // Reduce CAN status frame rates
            BaseStatusSignal.setUpdateFrequencyForAll(10, motor.getStickyFaultField()); //period 0

            BaseStatusSignal.setUpdateFrequencyForAll(50, motor.getPosition(), motor.getVelocity(),  motor.getRotorPosition()); //period 1
        
            BaseStatusSignal.setUpdateFrequencyForAll(1, motor.getSupplyCurrent(), motor.getStatorCurrent(), motor.getMotorVoltage(), motor.getSupplyVoltage()); //period 2
            // Optimize bus utilization
            motor.optimizeBusUtilization(1.0);

            //Did not set Conversion factors if needed add later
                        
            return new ControllerImplementation(motor, absoluteEncoder, sensorPositionCoefficient, sensorVelocityCoefficient);
        }
    }

    private class ControllerImplementation implements SteerController {
        private static final int ENCODER_RESET_ITERATIONS = 500;
        private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);
        
        private final TalonFX motor;
        private final AbsoluteEncoder absoluteEncoder;
        
        private final double sensorVelocityCoefficient;
        private final double positionConversionFactor;

        private double referenceAngleRadians = 0;

        private double resetIteration = 0;

        private ControllerImplementation(TalonFX motor, AbsoluteEncoder absoluteEncoder, double sensorPositionCoefficient, double sensorVelocityCoefficient) {
            this.motor = motor;
            this.absoluteEncoder = absoluteEncoder;
            this.sensorVelocityCoefficient = sensorVelocityCoefficient;
            this.positionConversionFactor = sensorPositionCoefficient;
        }
        


        @Override
        public TalonFX getSteerMotor() {
            return this.motor;
        }

        @Override
        public AbsoluteEncoder getSteerEncoder() {
            return this.absoluteEncoder;
        }

        @Override
        public double getReferenceAngle() {
            return referenceAngleRadians;
        }

        @Override
        public void setReferenceAngle(double referenceAngleRadians) {
            double currentAngleRadians = motor.getPosition().getValueAsDouble() * positionConversionFactor;
             // Reset the KrakenX44's encoder periodically when the module is not rotating.
            // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't
            // fully set up, and we don't
            // end up getting a good reading. If we reset periodically this won't matter
            // anymore.
            if (motor.getVelocity().getValueAsDouble() * sensorVelocityCoefficient < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
            // ConsoleLogger.consoleLog("Reset Iteration: "+resetIteration);
            if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
            // ConsoleLogger.consoleLog("resetIterationHit--");
                resetIteration = 0;
                double absoluteAngle = absoluteEncoder.getAbsoluteAngle();
                motor.setPosition(absoluteAngle);
                currentAngleRadians = absoluteAngle;
                currentAngleRadians=realign();
            }
            } else {
                resetIteration = 0;
            }

            double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);

            if (currentAngleRadiansMod < 0.0) {
                currentAngleRadiansMod += 2.0 * Math.PI;
            }

            // The reference angle has the range [0, 2pi) but the Neo's encoder can go above
            // that
            double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
            
            if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
                adjustedReferenceAngleRadians -= 2.0 * Math.PI;
            } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
                adjustedReferenceAngleRadians += 2.0 * Math.PI;
            }

            this.referenceAngleRadians = referenceAngleRadians;  
            PositionDutyCycle positionControl = new PositionDutyCycle(adjustedReferenceAngleRadians);  
            motor.setControl(positionControl);
          
        }

        @Override
        public double getStateAngle() {
            double motorAngleRadians = motor.getPosition().getValueAsDouble() * positionConversionFactor;
            motorAngleRadians %= 2.0 * Math.PI;
            if (motorAngleRadians < 0.0) {
                motorAngleRadians += 2.0 * Math.PI;
            }

            return motorAngleRadians;
        }

        @Override
        public double realign() {
            resetIteration = 0;
            double absoluteAngle = absoluteEncoder.getAbsoluteAngle();
            motor.setPosition(absoluteAngle);
            return absoluteAngle;
        }
    }
}