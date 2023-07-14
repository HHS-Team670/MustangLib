package frc.team670.mustanglib.commands.drive.teleop;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;

 

/**
 * Sets Swerve forward direction.
 * 
 * This Instant Command which implements MustandCommond so that is can be easily run by 
 * MustangScheduler as a part of MustangLib implementations in FRC code  from Team 670 is 
 * used to set the forward on a swerve drive that utilizes SDS swerve modules with two Neo 
 * motors. It does this by zeroing the heading on the gyroscope on the robot and then realigning 
 * the modules to the current heading so that the new fforward direction is the same as the current direction.
 */
public class SetSwerveForwardDirection extends InstantCommand implements MustangCommand{
    
    private SwerveDrive swerveDrive;
    private Map<MustangSubsystemBase, HealthState> healthRequirements = new HashMap<MustangSubsystemBase, HealthState>();
    
    public SetSwerveForwardDirection(SwerveDrive driveBase) {
        this.swerveDrive = driveBase;
        healthRequirements.put(swerveDrive, HealthState.YELLOW);

    }

    public void initialize() {
        swerveDrive.zeroHeading(); 
        swerveDrive.realignModules(); 
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthRequirements;
    }
}
