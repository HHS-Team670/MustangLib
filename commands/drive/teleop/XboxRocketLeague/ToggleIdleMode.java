package frc.team670.mustanglib.commands.drive.teleop.XboxRocketLeague;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.subsystems.drivebase.DriveBase;

public class ToggleIdleMode extends InstantCommand implements MustangCommand{

    private DriveBase driveBase;

    public ToggleIdleMode(DriveBase driveBase) {
        super();
        Logger.consoleLog("Switching idle mode");
        this.driveBase = driveBase;
    }

    public void initialize() {
        driveBase.toggleIdleMode();
      }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        // TODO Auto-generated method stub
        return null;
    }
    
}
