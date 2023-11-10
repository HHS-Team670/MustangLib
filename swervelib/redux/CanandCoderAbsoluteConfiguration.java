package frc.team670.mustanglib.swervelib.redux;

import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.reduxrobotics.sensors.canandcoder.CanandcoderSettings;

public class CanandCoderAbsoluteConfiguration extends CanandcoderSettings {
    private final int id;
   

    public CanandCoderAbsoluteConfiguration(int id) {
        this.id = id;
        
    }


    public int getId() {
        return id;
    }


}
