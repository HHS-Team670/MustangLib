package frc.team670.mustanglib.swervelib.redux;

import com.reduxrobotics.sensors.canandcoder.Canandcoder.Settings;

public class CanandCoderAbsoluteConfiguration extends Settings {
    private final int id;
   

    public CanandCoderAbsoluteConfiguration(int id) {
        this.id = id;
        
    }


    public int getId() {
        return id;
    }


}
