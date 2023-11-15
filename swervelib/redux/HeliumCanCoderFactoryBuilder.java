package frc.team670.mustanglib.swervelib.redux;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import frc.team670.mustanglib.swervelib.AbsoluteEncoder;
import frc.team670.mustanglib.swervelib.AbsoluteEncoderFactory;
import frc.team670.mustanglib.swervelib.ctre.CtreUtils;
import com.reduxrobotics.sensors.canandcoder.Canandcoder;
import com.reduxrobotics.sensors.canandcoder.CanandcoderFaults;
import com.reduxrobotics.sensors.canandcoder.CanandcoderSettings;

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
        }  

        @Override
        public double getAbsoluteAngle() {
            double angle = Math.toRadians(360/2/Math.PI*encoder.getPosition());
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
