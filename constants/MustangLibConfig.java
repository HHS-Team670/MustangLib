package frc.team670.mustanglib.constants;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;

public class MustangLibConfig {
    private static double leftKsVolts; 
    private static double leftKvVoltSecondsPerMeter; 
    private static double leftKaVoltSecondsSquaredPerMeter; 
    private static double rightKsVolts;
    private static double rightKvVoltSecondsPerMeter;
    private static double rightKaVoltSecondsSquaredPerMeter;
    private static DifferentialDriveKinematics kDriveKinematics;
    private static double kMaxSpeedMetersPerSecond;
    private static double kMaxAccelerationMetersPerSecondSquared;
    private static DifferentialDriveKinematicsConstraint kAutoPathConstraints;

    public MustangLibConfig (double leftKsVolts, double leftKvVoltSecondsPerMeter, double leftKaVoltSecondsSquaredPerMeter, 
    double rightKsVolts, double rightKvVoltSecondsPerMeter, double rightKaVoltSecondsSquaredPerMeter, 
    double kMaxSpeedMetersPerSecond, double kMaxAccelerationMetersPerSecondSquared, 
    DifferentialDriveKinematics kDriveKinematics, DifferentialDriveKinematicsConstraint kAutoPathConstraints) {
        this.leftKsVolts = leftKsVolts;
        this.leftKvVoltSecondsPerMeter = leftKvVoltSecondsPerMeter;
        this.leftKaVoltSecondsSquaredPerMeter = leftKaVoltSecondsSquaredPerMeter;
        this.rightKsVolts = rightKsVolts;
        this.rightKvVoltSecondsPerMeter = rightKvVoltSecondsPerMeter;
        this.rightKaVoltSecondsSquaredPerMeter = rightKaVoltSecondsSquaredPerMeter;
        this.kMaxSpeedMetersPerSecond = kMaxSpeedMetersPerSecond;
        this.kMaxAccelerationMetersPerSecondSquared = kMaxAccelerationMetersPerSecondSquared;
        this.kDriveKinematics = kDriveKinematics;
        this.kAutoPathConstraints = kAutoPathConstraints;
    }

    public double getLeftKsVolts() {
        return leftKsVolts;
    }

    public double getRightKsVolts() {
        return rightKsVolts;
    }

    public double getLeftKvVoltSecondsPerMeter() {
        return leftKvVoltSecondsPerMeter;
    }

    public double getRightKvVoltSecondsPerMeter() {
        return rightKvVoltSecondsPerMeter;
    }

    public double getLeftKaVoltSecondsSquaredPerMeter() {
        return leftKaVoltSecondsSquaredPerMeter;
    }

    public double getRightKaVoltSecondsSquaredPerMeter() {
        return rightKaVoltSecondsSquaredPerMeter;
    }

    public double getKMaxSpeedMetersPerSecond() {
        return kMaxSpeedMetersPerSecond;
    }

    public double getKMaxAccelerationMetersPerSecondSquared() {
        return kMaxAccelerationMetersPerSecondSquared;
    }

    public DifferentialDriveKinematics getKDriveKinematics() {
        return kDriveKinematics;
    }

    public DifferentialDriveKinematicsConstraint getKAutoPathConstraints() {
        return kAutoPathConstraints;
    }
}