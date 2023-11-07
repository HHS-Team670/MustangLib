package frc.team670.mustanglib.swervelib.redux;

import com.ctre.phoenix.sensors.SensorInitializationStrategy;

public class HeliumCanCoderAbsoluteConfiguration {
    private final int id;
    private final double offset;
    private final String canbus;
    private final SensorInitializationStrategy initStrategy;

    public HeliumCanCoderAbsoluteConfiguration(int id, double offset, String canbus, SensorInitializationStrategy initStrategy) {
        this.id = id;
        this.offset = offset;
        this.canbus = canbus;
        this.initStrategy = initStrategy;
    }

    public HeliumCanCoderAbsoluteConfiguration(int id, double offset, String canbus) {
        this(id, offset, canbus, SensorInitializationStrategy.BootToAbsolutePosition);
    }

    public HeliumCanCoderAbsoluteConfiguration(int id, double offset) {
        this(id, offset, "");
    }

    public int getId() {
        return id;
    }

    public double getOffset() {
        return offset;
    }

    public String getCanbus() {
        return canbus;
    }

    public SensorInitializationStrategy getInitStrategy() {
        return initStrategy;
    }
}
