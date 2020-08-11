package frc.team670.robot.commands;

import frc.team670.robot.subsystems.MustangSubsystemBase;

import java.util.Map;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

/**
 * Represents a robot action with defined health requirements for every subsystem it uses.
 * 
 * @author ctychen, lakshbhambhani
 */
public interface MustangCommand{

    /**
     * @return A Map containing the minimum health condition for each subsystem that this Command requires to be safely used.
     */
    public Map<MustangSubsystemBase, MustangSubsystemBase.HealthState> getHealthRequirements();

}