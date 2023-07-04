package frc.team670.mustanglib.swervelibold;

public interface DriveController {
    Object getDriveMotor();

    void setReferenceVoltage(double voltage);

    double getStateVelocity();

    double getDistanceMoved();
}
