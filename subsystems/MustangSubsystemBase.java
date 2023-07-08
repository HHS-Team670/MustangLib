package frc.team670.mustanglib.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;


import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
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

    private LoggableInputs inputs;
    private MustangSubsystemBaseIO io;
    private boolean failedLastTime=false;




    /**
     * Creates a new MustangSubsystemBase. By default, the subsystem's initial
     * health state is UNKNOWN (ID 0).
     */
    public MustangSubsystemBase(MustangSubsystemBaseIO io, LoggableInputs inputs) {
        this.io=io;
        this.inputs=inputs;
        
        // RobotContainer.addSubsystem(this);
        // this.lastHealthState = HealthState.UNKNOWN;
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
        return io.getHealth(check);
    }
    protected LoggableInputs getInputs(){
        return inputs;
    }

   

    

    public void initDefaultCommand(MustangCommand command) {
        MustangScheduler.getInstance().setDefaultCommand(this, command);
    }

    /**
     * Checks the health of this subsystem and attempts to run this subsystem's mustangperiodic if the health is yellow unknown or green
     */
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        io.getHealth(true);
        Logger.getInstance().processInputs(getName(), inputs);
        if (io.getHealth(false) == HealthState.GREEN || io.getHealth(false) == HealthState.UNKNOWN || io.getHealth(false) == HealthState.YELLOW) {
            if (failedLastTime) {
                MustangNotifications
                        .notify("Health state for " + this.getName() + " is: " + io.getHealth(false) + ". Enabling Periodic");
                
            }
            mustangPeriodic();
            
        } else {
            if (!failedLastTime) {
                MustangNotifications.reportError(
                        "Health state for " + this.getName() + " is: " + io.getHealth(false) + ". Disabling Periodic");
                
            }
        }
        if(io.shouldDebugOutputs()){
            io.debugOutputs();
        }
        
    }
    /**
     * 
     * @return the default command of this subsystem
     */
    public MustangCommand getDefaultMustangCommand() {
        return (MustangCommand) (super.getDefaultCommand());
    }
    /**
     * Function that runs periodically
     */
    public abstract void mustangPeriodic();

    
}
