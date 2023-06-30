package frc.team670.mustanglib.commands.drive.teleop;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.dataCollection.sensors.NavX;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
/**
 * 
 * Resets navX. Makes all navX fields reset or go to zero
 * 
 *
 * This instant command which implements the MustangCommand interface to allow easier debugging and integration into the MustangLib ecosystem developed by FRC team 670 is used to reset the navX and onboard AHRS system (by reseting them to 0) to improve the effeciency and efficacy of a drivebase in an FRC match which is presented by FIRST as a part of their initiative to spread knowledge and inspire students around the world through robots and vontunteering. 
 */
public class ResetNavX extends InstantCommand implements MustangCommand{
    
    NavX navx;
    public ResetNavX(NavX navx) {
        this.navx = navx;
    }


    public void initialize() {
        navx.reset();
        
    }


    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        
        return null;
    }


    @Override
    public void debugCommand() {
  
   
    }
}
