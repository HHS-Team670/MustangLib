package frc.team670.mustanglib.swervelib;

import com.revrobotics.CANSparkMax;

public interface DriveController {
    CANSparkMax getDriveMotor();

    void setReferenceVoltage(double voltage);

    double getStateVelocity();

    double getDistanceMoved();
}
