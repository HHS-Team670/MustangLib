package frc.team670.mustanglib.commands;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team670.mustanglib.utils.MustangNotifications;
import frc.team670.mustanglib.RobotBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.ConsoleLogger;

import java.util.Arrays;
import java.util.Map;

/**
 * Responsible for scheduling and running commands, including MustangCommands. 
 * Based on CommandScheduler, but uses health system. 
 *
 * Note: Although it makes sense for MustangScheduler to extend CommandScheduler,
 * it does not because it is not possible to cleanly implement the health system
 * doing so.
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
        //Checks for nulls
        int nl=0;//new length
        int count=0;
        for(int i=0;i<commands.length;i++){
            if(commands[i]==null){
                count++;
            }else{
                commands[i-count]=commands[i];
                nl++;
            }

        }
        commands=Arrays.copyOf(commands, nl);

        
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
            MustangNotifications.reportMinorWarning("Scheduler run without any command");
            return;
        }

        for (MustangCommand a_command : commands) {

            Command m_command = (Command) a_command;
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
                                RobotBase.getInstance().getRobotContainer().getDriverController().rumble(0.75, 1);
                                scheduleOrCancel(m_command);
                                return;
                            }
                        }
                    }
                }
                this.currentCommand = m_command;
                scheduler.schedule(currentCommand);
                ConsoleLogger.consoleLog("Command scheduled: %s", this.currentCommand.getName());
            } finally {
                this.currentCommand = null;
            }
        }
    }

    /**
     * To be used only as a convenience feature for quickly scheduling command groups. Expects nothing less than a green health
     * state for the subsystem
     * @param group The command group that needs to be scheduled
     * @param subsystem The subsystem being used for the command group
     */
    public void schedule(Command group, MustangSubsystemBase subsystem) {

        if (group == null) {
            MustangNotifications.reportMinorWarning("Scheduler run without any command");
            return;
        }

        HealthState currentHealth = subsystem.getHealth(false);
        if (currentHealth.getId() > HealthState.GREEN.getId()) {
            MustangNotifications.reportWarning(
                    "%s not run because of health issue! Required health: %s, Actual health: %s",
                    group.getName(), HealthState.GREEN, currentHealth);
            return;
        }

        this.currentCommand = group;
        scheduler.schedule(currentCommand);
        ConsoleLogger.consoleLog("Command scheduled: %s", this.currentCommand.getName());
        
    }

    /**
     * Initial check that runs when the command initializes to check if it is a
     * MustangCommand that has been scheduled using MustangScheduler
     * 
     * @param command command The command which has been scheduled
     */
    public void check(Command command) throws RuntimeException {
        if (command == null) {
            ConsoleLogger.consoleLog("Command is null");
            return;
        } else {
            if (!(command instanceof MustangCommand)) {
                MustangNotifications.reportError("%s was not properly scheduled. Are you using MustangScheduler?",
                        command.getName());
            }
        }
    }

    public void setDefaultCommand(MustangSubsystemBase subsystem, MustangCommand mCommand) {
        Command m_command = (Command) mCommand;
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
                                RobotBase.getInstance().getRobotContainer().getDriverController().rumble(0.75, 1);
                            scheduleOrCancel(m_command);
                            return;
                        }
                    }
                }
            }
            this.currentCommand = m_command;
            scheduler.setDefaultCommand(subsystem, currentCommand);
            ConsoleLogger.consoleLog("Command scheduled: %s", this.currentCommand.getName());
        } finally {
            this.currentCommand = null;
        }
    }

    public void unregisterSubsystem(MustangSubsystemBase... subsystems) {
        scheduler.unregisterSubsystem(subsystems);
    }

    public void registerSubsystem(MustangSubsystemBase... subsystems) {
        scheduler.registerSubsystem(subsystems);
    }

    public void scheduleOrCancel(Command command) {
        if (RobotBase.getInstance().getRobotContainer().getDriverController().getRightJoystickButton() == true) {
            scheduler.schedule(command);
        }
    }
}