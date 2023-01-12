package frc.team670.mustanglib.swervelib;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import frc.team670.mustanglib.utils.Logger;

@FunctionalInterface
public interface DriveControllerFactory<Controller extends DriveController, DriveConfiguration> {
    default void addDashboardEntries(
            ShuffleboardContainer container,
            Controller controller
    ) {
        container.addNumber("Current Velocity", controller::getStateVelocity);
    }

    default Controller create(
            ShuffleboardContainer container,
            DriveConfiguration driveConfiguration,
            String canbus,
            ModuleConfiguration moduleConfiguration
    ) {
        var controller = create(driveConfiguration, canbus, moduleConfiguration);
        addDashboardEntries(container, controller);

        return controller;
    }

    default Controller create(
            ShuffleboardContainer container,
            DriveConfiguration driveConfiguration,
            ModuleConfiguration moduleConfiguration
    ) {
        var controller = create(driveConfiguration, moduleConfiguration);
        addDashboardEntries(container, controller);

        return controller;
    }

    default Controller create(
        DriveConfiguration driveConfiguration,
        ModuleConfiguration moduleConfiguration
    ) {
        return create(driveConfiguration, "", moduleConfiguration);
    }

    Controller create(DriveConfiguration driveConfiguration, String canbus, ModuleConfiguration moduleConfiguration);
}
