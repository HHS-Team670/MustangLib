package frc.team670.mustanglib.swervelib.redux;

import com.reduxrobotics.sensors.canandmag.Canandmag.Settings;

public class CanandMagAbsoluteConfiguration extends Settings {
    private final int id;
   

    public CanandMagAbsoluteConfiguration(int id) {
        this.id = id;
        
    }


    public int getId() {
        return id;
    }


}
