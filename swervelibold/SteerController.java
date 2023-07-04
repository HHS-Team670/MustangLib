package frc.team670.mustanglib.swervelibold;

public interface SteerController {
    Object getSteerMotor();

    AbsoluteEncoder getSteerEncoder();

    double getReferenceAngle();

    void setReferenceAngle(double referenceAngleRadians);

    double getStateAngle();
    
    public double realign();
}
