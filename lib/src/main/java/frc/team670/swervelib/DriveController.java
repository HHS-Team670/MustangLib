package frc.team670.mustanglib.swervelib;

public interface DriveController {
    Object getDriveMotor();

    void setReferenceVoltage(double voltage);

    double getStateVelocity();

    double getDistanceMoved();
}
