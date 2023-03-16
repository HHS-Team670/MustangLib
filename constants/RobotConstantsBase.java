package frc.team670.mustanglib.constants;

public class RobotConstantsBase {

    /**
     * The number of ticks per rotation of a drivebase wheel for the DIO Encoders
     */
    public static final int DIO_TICKS_PER_ROTATION = 1024;

    /**
     * The number of ticks per rotation of a drivebase wheel for the SPARK Encoders
     */
    public static final int SPARK_TICKS_PER_ROTATION = 1024;

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = .7;

    public static final double LOOP_TIME = 0.015;

    public static final int VISION_ERROR_CODE = -99999;

}