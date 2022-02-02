package frc.team670.mustanglib.commands.drive.teleop;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.dataCollection.sensors.NavX;

public class ResetNavX extends InstantCommand{
    
    NavX navx;
    public ResetNavX(NavX navx) {
        this.navx = navx;
    }


    public void initialize() {
        navx.reset();
        
    }
}
