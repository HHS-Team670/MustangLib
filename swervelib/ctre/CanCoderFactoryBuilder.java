package frc.team670.mustanglib.swervelib.ctre;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.swervelib.AbsoluteEncoder;
import frc.team670.mustanglib.swervelib.AbsoluteEncoderFactory;

public class CanCoderFactoryBuilder {

    private int periodMilliseconds = 10;

    public CanCoderFactoryBuilder withReadingUpdatePeriod(int periodMilliseconds) {
        this.periodMilliseconds = periodMilliseconds;
        return this;
    }

    public AbsoluteEncoderFactory<CanCoderAbsoluteConfiguration> build() {
        return configuration -> {
            CANcoderConfiguration config = new CANcoderConfiguration();
            CANcoder encoder = new CANcoder(configuration.getId(), configuration.getCanbus());
            
            
            StatusCode val = encoder.getConfigurator().refresh(config);

            if(val.isOK()){
                  val = encoder.getConfigurator().refresh(config);
            }

            
            config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
            // config.MagnetSensor.MagnetOffset = configuration.getOffset();// Should be roations
            config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

            encoder.getConfigurator().apply(config);
            CtreUtils.checkCtreError(encoder.getConfigurator().apply(config), "Failed to configure CANCoder");

            // CtreUtils.checkCtreError(encoder.optimizeBusUtilization(periodMilliseconds), "Failed to configure CANCoder update rate");

            return new EncoderImplementation(encoder);
        };

    }

    private static class EncoderImplementation implements AbsoluteEncoder {
        private final int ATTEMPTS = 3;

        private final CANcoder encoder;

        private EncoderImplementation(CANcoder encoder) {
            this.encoder = encoder;
        }

        /**
         * @return the angle in radians 
         */
        @Override
        public double getAbsoluteAngle() {
            StatusSignal<Double> poStatusSignal = encoder.getAbsolutePosition();
            SmartDashboard.putNumber("Absolute Encoder Position", poStatusSignal.getValueAsDouble());
            double angle = 2 * Math.PI * poStatusSignal.getValue();

            StatusCode code = poStatusSignal.getStatus();

            for (int i = 0; i < ATTEMPTS; i++) {
                if (code == StatusCode.OK)
                    break;
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                }
                poStatusSignal = encoder.getAbsolutePosition();
                angle = 2 * Math.PI * poStatusSignal.getValue();

                code = poStatusSignal.getStatus();
                

            }

            CtreUtils.checkCtreError(code, "Failed to retrieve CANcoder " + encoder.getDeviceID()
                    + " absolute position after " + ATTEMPTS + " tries");

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
}
