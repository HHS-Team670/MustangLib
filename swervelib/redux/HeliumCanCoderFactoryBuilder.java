package frc.team670.mustanglib.swervelib.redux;


import frc.team670.mustanglib.swervelib.AbsoluteEncoder;
import frc.team670.mustanglib.swervelib.AbsoluteEncoderFactory;
import frc.team670.mustanglib.swervelib.ctre.CtreUtils;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.reduxrobotics.sensors.canandcoder.Canandcoder;
import com.reduxrobotics.sensors.canandcoder.Canandcoder.Faults;
import com.reduxrobotics.sensors.canandcoder.Canandcoder.Settings;

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
           

            Canandcoder encoder = new Canandcoder(configuration.getId());
            Settings settings = encoder.getSettings().setInvertDirection(true);
            encoder.setSettings(settings);
            return new EncoderImplementation(encoder);
        };
    }

    private static class EncoderImplementation implements AbsoluteEncoder {
        private final int ATTEMPTS = 3;

        private final Canandcoder encoder;

        private EncoderImplementation(Canandcoder encoder) {
            this.encoder = encoder;
        }  

        @Override
        public double getAbsoluteAngle() {
            double angle = 2*Math.PI*encoder.getAbsPosition();
            SmartDashboard.putNumber("HeliumPos", angle);
            // CanandcoderFaults code = encoder.getActiveFaults();

            // for (int i = 0; i < ATTEMPTS; i++) {
            //     if (!code.faultsValid()) break; //note disable if this breaks stuff
            //     try {
            //         Thread.sleep(10);
            //     } catch (InterruptedException e) { }
            //     angle = Math.toRadians(encoder.getPosition());
            //     code = encoder.getActiveFaults();
            // }

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
