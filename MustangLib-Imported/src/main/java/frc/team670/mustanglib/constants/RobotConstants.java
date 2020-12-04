package frc.team670.mustanglib.constants;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;

public class RobotConstants {

    /**
     * The number of ticks per rotation of a drivebase wheel for the DIO Encoders
     */
    public static final int DIO_TICKS_PER_ROTATION = 1024;

    /**
     * The number of ticks per rotation of a drivebase wheel for the SPARK Encoders
     */
    public static final int SPARK_TICKS_PER_ROTATION = 1024;

	public static final TrajectoryConstraint kAutoPathConstraints = null;

	public static final double ksVolts = 0;

	public static final double kvVoltSecondsPerMeter = 0;

	public static final double kaVoltSecondsSquaredPerMeter = 0;

	public static final DifferentialDriveKinematics kDriveKinematics = null;

	public static final double kMaxSpeedMetersPerSecond = 0;

	public static final double kMaxAccelerationMetersPerSecondSquared = 0;

	public static final boolean kNavXReversed = false;


}