package frc.team670.mustanglib.swervelib;

import frc.team670.mustanglib.swervelib.ctre.*;
import frc.team670.mustanglib.swervelib.rev.*;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class Mk4SwerveModuleBuilder {

    public enum GearRatio {
        L1(SdsModuleConfigurations.MK4_L1),
        L2(SdsModuleConfigurations.MK4_L2),
        L3(SdsModuleConfigurations.MK4_L3),
        L4(SdsModuleConfigurations.MK4_L4);

        private final ModuleConfiguration configuration;

        GearRatio(ModuleConfiguration configuration) {
            this.configuration = configuration;
        }

        public ModuleConfiguration getConfiguration() {
            return configuration;
        }
    }

    private static DriveControllerFactory<?, Integer> getKrakenX60DriveFactory(Mk4ModuleConfiguration configuration) {
        return new KrakenX60DriveControllerFactoryBuilder()
                .withVoltageCompensation(configuration.getNominalVoltage())
                .withCurrentLimit(configuration.getDriveCurrentLimit())
                .build();
    }

    private static DriveControllerFactory<?, Integer> getNeoDriveFactory(Mk4ModuleConfiguration configuration) {
        return new NeoDriveControllerFactoryBuilder()
                .withVoltageCompensation(configuration.getNominalVoltage())
                .withCurrentLimit(configuration.getDriveCurrentLimit())
                .build();
    }

    private static SteerControllerFactory<?, SteerConfiguration<CanCoderAbsoluteConfiguration>> getNeoSteerFactory(Mk4ModuleConfiguration configuration) {
        return new NeoSteerControllerFactoryBuilder()
                .withVoltageCompensation(configuration.getNominalVoltage())
                .withPidConstants(1.0, 0.0, 0.1)
                .withCurrentLimit(configuration.getSteerCurrentLimit())
                .build(new CanCoderFactoryBuilder()
                        .withReadingUpdatePeriod(100)
                        .build());
    }

    private final Mk4ModuleConfiguration configuration;
    private ShuffleboardLayout container = null;
    private GearRatio gearRatio = null;

    private DriveControllerFactory<?, Integer> driveFactory = null;
    private SteerControllerFactory<?, SteerConfiguration<CanCoderAbsoluteConfiguration>> steerFactory = null;

    private int driveMotorPort = -1;
    private String driveCanbus = "";
    private int steerMotorPort = -1;
    private String steerCanbus = "";

    private MotorType steerMotorType;
    private int steerEncoderPort = -1;
    private double steerOffset = 0;
    private String steerEncoderCanbus = "";

    public Mk4SwerveModuleBuilder() {
        this.configuration = new Mk4ModuleConfiguration();
    }

    public Mk4SwerveModuleBuilder(Mk4ModuleConfiguration configuration) {
        this.configuration = configuration;
    }

    public Mk4SwerveModuleBuilder withLayout(ShuffleboardLayout container) {
        this.container = container;
        return this;
    }

    public Mk4SwerveModuleBuilder withGearRatio(GearRatio gearRatio) {
        this.gearRatio = gearRatio;
        return this;
    }

    public Mk4SwerveModuleBuilder withDriveMotor(MotorType motorType, int motorPort, String motorCanbus) {
        switch (motorType) {
            case NEO:
                this.driveFactory = getNeoDriveFactory(this.configuration);
                break;
            case KRAKEN_X60:
                this.driveFactory = getKrakenX60DriveFactory(this.configuration);
                break;
            default:
                break;
        }
        this.driveMotorPort = motorPort;
        this.driveCanbus = motorCanbus;
        return this;
    }

    public Mk4SwerveModuleBuilder withDriveMotor(MotorType motorType, int motorPort) {
        return this.withDriveMotor(motorType, motorPort, "");
    }

    public Mk4SwerveModuleBuilder withSteerMotor(MotorType motorType, int motorPort, String motorCanbus) {
        switch (motorType) {
            case NEO:
                this.steerFactory = getNeoSteerFactory(this.configuration);
                break;
            default:
                break;
        }
        this.steerMotorType = motorType;
        this.steerMotorPort = motorPort;
        this.steerCanbus = motorCanbus;
        return this;
    }

    public Mk4SwerveModuleBuilder withSteerMotor(MotorType motorType, int motorPort) {
        return this.withSteerMotor(motorType, motorPort, "");
    }

    public Mk4SwerveModuleBuilder withSteerEncoderPort(int encoderPort, String canbus) {
        this.steerEncoderPort = encoderPort;
        this.steerEncoderCanbus = canbus;
        return this;
    }

    public Mk4SwerveModuleBuilder withSteerEncoderPort(int encoderPort) {
        return this.withSteerEncoderPort(encoderPort, "");
    }

    public Mk4SwerveModuleBuilder withSteerOffset(double offset) {
        this.steerOffset = offset;
        return this;
    }

    public SwerveModule build() {
        if (gearRatio == null) {
            throw new RuntimeException("Gear Ratio should not be null!");
        }

        if (driveFactory == null) {
            throw new RuntimeException("Drive Motor should not be null!");
        }

        if (steerFactory == null) {
            throw new RuntimeException("Steer Motor should not be null!");
        }

        if (driveMotorPort < 0) {
            throw new RuntimeException("Drive Motor Port should be greater than 0!");
        }

        if (steerMotorPort < 0) {
            throw new RuntimeException("Steer Motor Port should be greater than 0!");
        }

        if (steerEncoderPort < 0) {
            throw new RuntimeException("Steer Encoder Port should be greater than 0!");
        }

        SwerveModuleFactory<Integer, SteerConfiguration<CanCoderAbsoluteConfiguration>> factory = new SwerveModuleFactory<>(
                gearRatio.getConfiguration(), 
                driveFactory, 
                steerFactory
        );

        SteerConfiguration<CanCoderAbsoluteConfiguration> steerConfig;

        if (steerMotorType == MotorType.KRAKEN_X60) {
            steerConfig = new SteerConfiguration<>(
                    steerMotorPort, 
                    new CanCoderAbsoluteConfiguration(
                            steerEncoderPort, 
                            steerEncoderCanbus
                    )
            );
        } else {
            throw new RuntimeException("Steer Motor Type should not be null!");
        }

        if (container == null) {
            return factory.create(
                    driveMotorPort, 
                    driveCanbus, 
                    steerConfig, 
                    steerCanbus
            );
        } else {
            return factory.create(
                    container, 
                    driveMotorPort, 
                    driveCanbus, 
                    steerConfig, 
                    steerCanbus
            );
        }
    }
}
