package frc.team670.mustanglib.swervelib;

import frc.team670.mustanglib.swervelib.ctre.*;
import frc.team670.mustanglib.swervelib.redux.*;
import frc.team670.mustanglib.swervelib.rev.*;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public final class Mk4iSwerveModuleHelper {
    private Mk4iSwerveModuleHelper() {
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

    
  

   

  

    /**
     * Creates a Mk4i swerve module that uses NEOs for driving and steering.
     * Module information is displayed in the specified ShuffleBoard container.
     *
     * @param container        The container to display module information in.
     * @param configuration    Module configuration parameters to use.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive NEO.
     * @param steerMotorPort   The CAN ID of the steer NEO.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @param encoderType      The type of encoder this module is using
     * @return The configured swerve module.
     */
    public static SwerveModule createNeo(
            ShuffleboardLayout container,
            Mk4ModuleConfiguration configuration,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset,
            AbsoluteEncoderType encoderType
    ) {
        if(encoderType == AbsoluteEncoderType.HELIUMCANCODER){
                return new SwerveModuleFactory<>(
                        gearRatio.getConfiguration(),
                        getNeoDriveFactory(configuration),
                        getNeoSteerFactory(configuration)
                ).create(
                        container,
                        driveMotorPort,
                        new SteerConfiguration<>(
                                steerMotorPort,
                                new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset)
                        )
                );
        }
        return new SwerveModuleFactory<>(
                gearRatio.getConfiguration(),
                getNeoDriveFactory(configuration),
                getNeoSteerFactory(configuration)
        ).create(
                container,
                driveMotorPort,
                new SteerConfiguration<>(
                        steerMotorPort,
                        new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset)
                )
        );

    }

    /**
     * Creates a Mk4i swerve module that uses NEOs for driving and steering.
     * Module information is displayed in the specified ShuffleBoard container.
     *
     * @param container        The container to display module information in.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive NEO.
     * @param steerMotorPort   The CAN ID of the steer NEO.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @param encoderType      The type of encoder this module is using
     * @return The configured swerve module.
     */
    public static SwerveModule createNeo(
            ShuffleboardLayout container,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset,
            AbsoluteEncoderType encoderType
    ) {
        return createNeo(container, new Mk4ModuleConfiguration(), gearRatio, driveMotorPort, steerMotorPort, steerEncoderPort, steerOffset, encoderType);
    }

    /**
     * Creates a Mk4i swerve module that uses NEOs for driving and steering.
     *
     * @param configuration    Module configuration parameters to use.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive NEO.
     * @param steerMotorPort   The CAN ID of the steer NEO.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @param encoderType      The type of encoder this module is using
     * @return The configured swerve module.
     */
    public static SwerveModule createNeo(
            Mk4ModuleConfiguration configuration,
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset,
            AbsoluteEncoderType encoderType
    ) {
        if(encoderType == AbsoluteEncoderType.HELIUMCANCODER){
                return new SwerveModuleFactory<>(
                        gearRatio.getConfiguration(),
                        getNeoDriveFactory(configuration),
                        getNeoSteerFactory(configuration)
                ).create(
                        driveMotorPort,
                        new SteerConfiguration<>(
                                steerMotorPort,
                                new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset)
                        )
                );
        }
        return new SwerveModuleFactory<>(
                gearRatio.getConfiguration(),
                getNeoDriveFactory(configuration),
                getNeoSteerFactory(configuration)
        ).create(
                driveMotorPort,
                new SteerConfiguration<>(
                        steerMotorPort,
                        new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset)
                )
        );
    }

    /**
     * Creates a Mk4i swerve module that uses NEOs for driving and steering.
     *
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive NEO.
     * @param steerMotorPort   The CAN ID of the steer NEO.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @param encoderType      The type of encoder this module is using
     * @return The configured swerve module.
     */
    public static SwerveModule createNeo(
            GearRatio gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset,
            AbsoluteEncoderType encoderType
    ) {
        return createNeo(new Mk4ModuleConfiguration(), gearRatio, driveMotorPort, steerMotorPort, steerEncoderPort, steerOffset, encoderType);
    }

    

    public enum GearRatio {
        L1(SdsModuleConfigurations.MK4I_L1),
        L2(SdsModuleConfigurations.MK4I_L2),
        L3(SdsModuleConfigurations.MK4I_L3);

        public final ModuleConfiguration configuration;

        GearRatio(ModuleConfiguration configuration) {
            this.configuration = configuration;
        }

        public ModuleConfiguration getConfiguration() {
            return configuration;
        }
    }
}
