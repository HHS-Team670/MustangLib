package frc.team670.mustanglib.swervelib;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModule {
    CANSparkMax getDriveMotor();

    CANSparkMax getSteerMotor();

    AbsoluteEncoder getSteerEncoder();

    double getDriveVelocity();

    double getSteerAngle();

    void set(double driveVoltage, double steerAngle);

    public void realign();

    public SwerveModuleState getState();

    public SwerveModulePosition getPosition();
}
