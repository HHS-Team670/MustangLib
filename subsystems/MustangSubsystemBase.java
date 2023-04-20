package frc.team670.mustanglib.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.utils.MustangNotifications;

/**
 * Basic framework for a subsystem of the robot with defined levels of system
 * Health. MustangSubsystems are state machines with a target state and an
 * actual state; its state of health affects what Commands the Robot is able to
 * run. Each MustangSubsystem is responsible for instantiating its components,
 * as well as running a routine to zero any sensors it requires.
 * 
 * @author ctychen
 */
public abstract class MustangSubsystemBase extends SubsystemBase {

    protected HealthState lastHealthState;
    private boolean failedLastTime = false;

    private static NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private static NetworkTable table = instance.getTable("/SmartDashboard");

    private boolean debugSubsystemFields = false;

    /**
     * Creates a new MustangSubsystemBase. By default, the subsystem's initial
     * health state is UNKNOWN (ID 0).
     */
    public MustangSubsystemBase() {
        // RobotContainer.addSubsystem(this);
        this.lastHealthState = HealthState.UNKNOWN;
    }

    /**
     * Represents possible conditions a MustangSubsystemBase can be in. Each
     * MustangSubsystemBase should define what the States mean for it specifically.
     * The default state is UNKNOWN, before the subsystem is first "used".
     */
    public enum HealthState {
        UNKNOWN(0), GREEN(1), YELLOW(2), RED(3);

        private final int ID;

        HealthState(int id) {
            ID = id;
        }

        /**
         * Gets the ID of the state.
         */
        public int getId() {
            return ID;
        }

    }

    /**
     * 
     * @param check Whether or not the subsystem's health should be (re)calculated.
     *              If false, this method simply returns the last recorded health
     *              state. If true, the method will re-evaluate the subsystem's
     *              current health. Note that before the first time the subsystem is
     *              "used", by default its state is UNKNOWN, and thus its health
     *              will be calculated at this time.
     * @return The latest known state of this subsystem: GREEN, YELLOW, or RED.
     */
    public HealthState getHealth(boolean check) {
        if (lastHealthState == HealthState.UNKNOWN || check) {
            lastHealthState = checkHealth();
        }
        return this.lastHealthState;
    }

    public void setDebugSubsystem(boolean toggle) {
        this.debugSubsystemFields = toggle;
    }

    /**
     * Calculates the current state of the subsystem.
     */
    public abstract HealthState checkHealth();

    public void initDefaultCommand(MustangCommand command) {
        CommandScheduler.getInstance().setDefaultCommand(this, (CommandBase) command);
    }

    @Override
    public void periodic() {
        HealthState lastHealth = getHealth(false);
        if (lastHealth == HealthState.GREEN || lastHealth == HealthState.UNKNOWN || lastHealth == HealthState.YELLOW) {
            if (failedLastTime) {
                MustangNotifications
                        .notify("Health state for " + this.getName() + " is: " + lastHealth + ". Enabling Periodic");
                failedLastTime = false;
            }
            mustangPeriodic();
            if(debugSubsystemFields){
                debugSubsystem();
            }
        } else {
            if (!failedLastTime) {
                MustangNotifications.reportError(
                        "Health state for " + this.getName() + " is: " + lastHealth + ". Disabling Periodic");
                failedLastTime = true;
            }
        }
    }

    public void pushHealthToDashboard() {
        NetworkTableEntry subsystem = table.getEntry(this.getName());
        subsystem.setString(getHealth(false).toString());
        if (getHealth(false).toString().equals("YELLOW") || getHealth(false).toString().equals("RED")) {
            // RobotContainer.notifyDriverController(1.0, 0.3);
        }
    }

    public MustangCommand getDefaultMustangCommand() {
        return (MustangCommand) (super.getDefaultCommand());
    }

    public abstract void mustangPeriodic();

    public abstract void debugSubsystem();
}