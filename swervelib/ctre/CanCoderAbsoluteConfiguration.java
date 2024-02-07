package frc.team670.mustanglib.swervelib.ctre;

public class CanCoderAbsoluteConfiguration {
    private final int id;
    private final String canbus;

    public CanCoderAbsoluteConfiguration(int id,  String canbus) {
        this.id = id;
        this.canbus = canbus;
    }

    public CanCoderAbsoluteConfiguration(int id) {
        this(id, "");
    }

    public int getId() {
        return id;
    }

    public String getCanbus() {
        return canbus;
    }
}
