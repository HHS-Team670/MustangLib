package frc.team670.mustanglib.commands.drive.teleop;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;

public class SetSwerveForwardDirection extends InstantCommand implements MustangCommand{
    
    private SwerveDrive swerveDrive;
    private Map<MustangSubsystemBase, HealthState> healthRequirements = new HashMap<MustangSubsystemBase, HealthState>();
    
    public SetSwerveForwardDirection(SwerveDrive driveBase) {
        this.swerveDrive = driveBase;
        healthRequirements.put(swerveDrive, HealthState.YELLOW);

    }

    public void initialize() {
        swerveDrive.zeroGyroscope(); 
        swerveDrive.realignModules(); 
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthRequirements;
    }
    @Override
    public void debugCommand(){}
}
