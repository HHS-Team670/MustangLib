package frc.team670.mustanglib.commands.drive.teleop;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;

public class SetSwerveForwardDirection extends InstantCommand implements MustangCommand{
    
    private SwerveDrive swerveDrive;

    public SetSwerveForwardDirection(SwerveDrive driveBase) {
        this.swerveDrive = driveBase;
    }

    public void initialize() {
        swerveDrive.zeroGyroscope(); 
        swerveDrive.realignModules(); 
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }
}
