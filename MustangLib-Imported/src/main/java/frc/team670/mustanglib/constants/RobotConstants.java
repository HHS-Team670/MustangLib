package frc.team670.mustanglib.constants;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;

public class RobotConstants {

    /**
     * The number of ticks per rotation of a drivebase wheel for the DIO Encoders
     */
    public static final int DIO_TICKS_PER_ROTATION = 1024;

    /**
     * The number of ticks per rotation of a drivebase wheel for the SPARK Encoders
     */
    public static final int SPARK_TICKS_PER_ROTATION = 1024;

    /**
     * Autonomous constants for two PID Controllers and two feedforwards
     */

    // "WHEEL_BASE" is really track width
    public static final double kTrackwidthMeters = 0.702;

    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackwidthMeters);
    public static final double leftKPDriveVel = 0;
    public static final double leftKIDriveVel = 0.00;
    public static final double leftKDDriveVel = 0.0;

    public static final double rightKPDriveVel = 0;
    public static final double rightKIDriveVel = 0.00;
    public static final double rightKDDriveVel = 0.0;

    public static final double rightKsVolts = 0;
    public static final double rightKvVoltSecondsPerMeter = 0;
    public static final double rightKaVoltSecondsSquaredPerMeter = 0;

    public static final double leftKsVolts = 0;
    public static final double leftKvVoltSecondsPerMeter = 0;
    public static final double leftKaVoltSecondsSquaredPerMeter = 0;

    public static final double kMaxSpeedMetersPerSecond = 0; 
    public static final double kMaxAccelerationMetersPerSecondSquared = 0;

    public static final DifferentialDriveKinematicsConstraint kAutoPathConstraints = new DifferentialDriveKinematicsConstraint(
            kDriveKinematics, kMaxSpeedMetersPerSecond);

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = .7;

}