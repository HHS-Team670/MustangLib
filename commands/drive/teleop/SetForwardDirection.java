package frc.team670.mustanglib.commands.drive.teleop;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;

public class SetForwardDirection extends InstantCommand implements MustangCommand{
    
    private SwerveDrive swerveDrive;

    public SetForwardDirection(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        
    }

    public void initialize() {
        swerveDrive.zeroGyroscope(); 
        swerveDrive.realignModules(); 
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }
}
