package frc.team670.mustanglib.swervelib.redux;


import frc.team670.mustanglib.swervelib.AbsoluteEncoder;
import frc.team670.mustanglib.swervelib.AbsoluteEncoderFactory;
import frc.team670.mustanglib.swervelib.ctre.CtreUtils;

import com.reduxrobotics.sensors.canandcoder.Canandcoder;
import com.reduxrobotics.sensors.canandcoder.Canandcoder.Faults;
import com.reduxrobotics.sensors.canandcoder.Canandcoder.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HeliumCanCoderFactoryBuilder {

    private Direction direction = Direction.COUNTER_CLOCKWISE;
    private int periodMilliseconds = 10;
     

    public HeliumCanCoderFactoryBuilder withReadingUpdatePeriod(int periodMilliseconds) {
        this.periodMilliseconds = periodMilliseconds;
        return this;
    }

    public HeliumCanCoderFactoryBuilder withDirection(Direction direction) {
        this.direction = direction;
        return this;
    }

    public AbsoluteEncoderFactory<CanandCoderAbsoluteConfiguration> build() {
        return configuration -> {
           

            Canandcoder encoder = new Canandcoder(configuration.getId());
          
            return new EncoderImplementation(encoder);
        };
    }

    private static class EncoderImplementation implements AbsoluteEncoder {
        private final int ATTEMPTS = 3;

        private final Canandcoder encoder;

        private EncoderImplementation(Canandcoder encoder) {
            this.encoder = encoder; 
            Settings settings = encoder.getSettings();
            // We are inverting the encoder because we are using Mk4i modules. If we use a different module, these may change
            settings.setInvertDirection(true);
            settings.setPositionFramePeriod(0.020);
            settings.setVelocityFramePeriod(0);
            encoder.setSettings(settings);
        }  

        @Override
        public double getAbsoluteAngle() {
            double angle = 2*Math.PI*encoder.getAbsPosition();
            SmartDashboard.putNumber("HeliumPos", angle);
          

   
            angle %= 2.0 * Math.PI;
            if (angle < 0.0) {
                angle += 2.0 * Math.PI;
            }

            return angle;

        }

        @Override
        public Object getInternal() {
            return this.encoder;
        }
    }
    

    public enum Direction {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    }
}
