package frc.team670.mustanglib.swervelib.pathplanner;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;

public class MustangPathPlannerAuto extends PathPlannerAuto implements MustangCommand {

    


    public MustangPathPlannerAuto(String autoName) {
        super(autoName);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return null;
    }
    
}
