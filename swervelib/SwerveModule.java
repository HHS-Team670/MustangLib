package frc.team670.mustanglib.swervelib;

public interface SwerveModule {
    Object getDriveMotor();

    Object getSteerMotor();

    AbsoluteEncoder getSteerEncoder();

    double getDriveVelocity();

    double getSteerAngle();

    void set(double driveVoltage, double steerAngle);

    public void realign();
}
