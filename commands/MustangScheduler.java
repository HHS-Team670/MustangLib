package frc.team670.mustanglib.commands;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DriverStation;

import frc.team670.mustanglib.utils.MustangNotifications;
import frc.team670.robot.RobotContainer;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.Logger;

import java.util.Arrays;
import java.util.Map;

/**
 * Responsible for scheduling and running commands, including
 * MustangCommandBases. Use this instead of the WPILib CommandScheduler.
 * 
 * @author ctychen, lakshbhambhani
 */

public class MustangScheduler {

    private Command currentCommand;

    private CommandScheduler scheduler;

    private static MustangScheduler instance;

    /**
     * Returns the MustangScheduler instance.
     *
     * @return the instance
     */
    public static synchronized MustangScheduler getInstance() {
        if (instance == null) {
            instance = new MustangScheduler();
        }
        return instance;
    }

    MustangScheduler() {
        scheduler = CommandScheduler.getInstance();
        scheduler.onCommandInitialize(command -> check(command));
    }

    public void run() {
        scheduler.run();
    }

    /**
     * Cancel the given MustangCommands. 
     */
    public void cancel(MustangCommand... commands) {
        // We can't directly cast an array of MustangCommands to an array of Commands, you
        // can only do that for subtype to supertype array. 
        // So each MustangCommand will be cast to a Command and added to an array of Commands.
        Command[] commandsToCancel = Arrays.copyOf(commands, commands.length, Command[].class);
        scheduler.cancel((Command[]) commandsToCancel);
    }

    public Command getCurrentlyScheduled() {
        return this.currentCommand;
    }

    /**
     * Cancel all currently scheduled commands.
     */
    public void cancelAll() {
        scheduler.cancelAll();
    }

    public void schedule(MustangCommand... commands) {

        if (commands == null) {
            MustangNotifications.reportMinorWarning("Scheduler run without any command"); // TODO Choose if we want to
                                                                                          // throw a warning or a minor
                                                                                          // warning which does not kill
                                                                                          // the jar
            return;
        }

        for (MustangCommand a_command : commands) {

            CommandBase m_command = (CommandBase) a_command;
            try {
                Map<MustangSubsystemBase, MustangSubsystemBase.HealthState> requirements = ((MustangCommand) (m_command))
                        .getHealthRequirements();

                if (requirements != null) {
                    for (MustangSubsystemBase s : requirements.keySet()) {
                        MustangSubsystemBase.HealthState healthReq = requirements.get(s);
                        if (s != null && healthReq != null) {
                            HealthState currentHealth = s.getHealth(false);
                            if (currentHealth.getId() > healthReq.getId()) {
                                MustangNotifications.reportWarning(
                                        "%s not run because of health issue! Required health: %s, Actual health: %s",
                                        m_command.getName(), healthReq, currentHealth);
                                RobotContainer.getDriverController().rumble(0.75, 1);
                                scheduleOrCancel(m_command);
                                return;
                            }
                        }
                    }
                }
                this.currentCommand = m_command;
                scheduler.schedule(currentCommand);
                Logger.consoleLog("Command scheduled: %s", this.currentCommand.getName());
            } finally {
                this.currentCommand = null;
            }
        }
    }

    /**
     * Initial check that runs when the command initializes to check if it is a
     * MustangCommand that has been scheduled using MustangScheduler
     * 
     * @param command command The command which has been scheduled
     */
    public void check(Command command) throws RuntimeException {
        if (command == null) {
            Logger.consoleLog("Command is null");
            return;
        } else {
            if (!(command instanceof MustangCommand)) {
                MustangNotifications.reportError("%s was not properly scheduled. Are you using MustangScheduler?",
                        command.getName());
            }
        }
    }

    public void setDefaultCommand(MustangSubsystemBase subsystem, MustangCommand command) {
        CommandBase m_command = (CommandBase) command;
        try {
            Map<MustangSubsystemBase, MustangSubsystemBase.HealthState> requirements = ((MustangCommand) (m_command))
                    .getHealthRequirements();

            if (requirements != null) {
                for (MustangSubsystemBase s : requirements.keySet()) {
                    MustangSubsystemBase.HealthState healthReq = requirements.get(s);
                    if (s != null && healthReq != null) {
                        HealthState currentHealth = s.getHealth(false);
                        if (currentHealth.getId() > healthReq.getId()) {
                            MustangNotifications.reportError(
                                    "%s not run because of health issue! Required health: %s, Actual health: %s",
                                    m_command.getName(), healthReq, currentHealth);
                            RobotContainer.getDriverController().rumble(0.75, 1);
                            scheduleOrCancel(m_command);
                            return;
                        }
                    }
                }
            }
            this.currentCommand = m_command;
            scheduler.setDefaultCommand(subsystem, currentCommand);
            Logger.consoleLog("Command scheduled: %s", this.currentCommand.getName());
        } finally {
            this.currentCommand = null;
        }
    }

    public void unregisterSubsystem(MustangSubsystemBase... subsystems){
        scheduler.unregisterSubsystem(subsystems);
    }

    public void registerSubsystem(MustangSubsystemBase... subsystems){
        scheduler.registerSubsystem(subsystems);
    }

    public void scheduleOrCancel (CommandBase command) {
    	if (RobotContainer.getDriverController().getRightJoystickButton() == true) {
    		scheduler.schedule(command);
        }
    }
}