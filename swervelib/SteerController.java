package frc.team670.mustanglib.swervelib;

import com.revrobotics.CANSparkMax;

import frc.team670.mustanglib.swervelib.encoder.AbsoluteEncoder;

public interface SteerController {
    CANSparkMax getSteerMotor();

    AbsoluteEncoder getSteerEncoder();

    double getReferenceAngle();

    void setReferenceAngle(double referenceAngleRadians);

    double getStateAngle();
    
    public double realign();
}
