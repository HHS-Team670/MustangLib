package frc.team670.mustanglib.swervelib.redux;


import frc.team670.mustanglib.swervelib.AbsoluteEncoder;
import frc.team670.mustanglib.swervelib.AbsoluteEncoderFactory;
import frc.team670.mustanglib.swervelib.ctre.CtreUtils;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.reduxrobotics.sensors.canandmag.Canandmag;
import com.reduxrobotics.sensors.canandmag.Canandmag.Faults;
import com.reduxrobotics.sensors.canandmag.Canandmag.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HeliumCanCoderFactoryBuilder {

    private Direction direction = Direction.CLOCKWISE;
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
            Canandmag encoder = new Canandmag(configuration.getId());
            // Settings settings = encoder.getSettings().setInvertDirection(true);
            // encoder.setSettings(settings);
            return new EncoderImplementation(encoder);
        };
    }

    private static class EncoderImplementation implements AbsoluteEncoder {
        private final int ATTEMPTS = 3;

        private final Canandmag encoder;

        private EncoderImplementation(Canandmag encoder) {
            this.encoder = encoder; 
            // Settings settings = new Settings();
            // // We are inverting the encoder because we are using Mk4i modules. If we use a different module, this may change
            // settings.setInvertDirection(true);
            // settings.setPositionFramePeriod(0.020);
            // settings.setVelocityFramePeriod(0);
            // encoder.setSettings(settings);
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
