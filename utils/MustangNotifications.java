package frc.team670.mustanglib.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Utility class used to send notifications to DriverStation, logs (Logger.consoleLog/consoleError), and NetworkTables.
 * @author lakshbhambhani
 */
public class MustangNotifications {

    private static NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private static NetworkTable table = instance.getTable("/SmartDashboard");
    private static NetworkTableEntry warning = table.getEntry("warnings");

    // While not at competition, some errors will kill robot code so we can debug.
    // If overrideAtCompetition is set to true, those errors will kill robot code
    // anyway, even if we're at a competition.
    private static boolean overrideAtCompetition;

    public MustangNotifications(boolean overrideAtCompetition){
        MustangNotifications.overrideAtCompetition = overrideAtCompetition;
    }

    /**
     * To check if the robot is atCompetition or getting tested
     * @return boolean atCompetition true if at competition
     */
    public static boolean atCompetition() {
        return (DriverStation.isFMSAttached() && !overrideAtCompetition);
    }

    /**
     * Used to report warnings to driverStation, Log, and FRCDashboard. Throws runtimeException to force the user to solve
     * the problem if not at competition
     * @param message Message with optional format specifiers for listed parameters. Use '%s' for formatting. It makes the parameters appear in the String where the '%s' are in order of input.
     * @param parameters Parameter list matching format specifiers
     */
    public static void reportWarning(String message, Object... parameters) {
        DriverStation.reportWarning(String.format(message, parameters), false);
        Logger.consoleWarning(message, parameters);
        warning.setString(String.format(message, parameters));
        if (!atCompetition()) //If not at competition, jar should be stopped to trace the problem and solve
            throw new RuntimeException(message);
    }

    /**
     * Used to report minor warnings to driverStation, Log, and FRCDashboard. Would not stop the jar during the test
     * @param message Message with optional format specifiers for listed parameters. Use '%s' for formatting. It makes the parameters appear in the String where the '%s' are in order of input.
     * @param parameters Parameter list matching format specifiers
     */
    public static void reportMinorWarning(String message, Object... parameters) {
        DriverStation.reportWarning(String.format(message, parameters), false);
        Logger.consoleWarning(message, parameters);
        warning.setString(String.format(message, parameters));
    }

    /**
     * Used to report Error to driverStation, Log, and FRCDashboard
     * @param message Message with optional format specifiers for listed parameters. Use '%s' for formatting. It makes the parameters appear in the String where the '%s' are in order of input.
     * @param parameters Parameter list matching format specifiers
     */
    public static void reportError(String message, Object... parameters) {
        DriverStation.reportError(String.format(message, parameters), false);
        Logger.consoleError(message, parameters);
        warning.setString(String.format(message, parameters));
        if (!atCompetition()) //If not at competition, jar should be stopped to trace the problem and solve
            throw new RuntimeException(message);
    }

    /**
     * Used to send notifications to driverStation, Log, and FRCDashboard
     * @param message Message with optional format specifiers for listed parameters. Use '%s' for formatting. It makes the parameters appear in the String where the '%s' are in order of input.
     * @param parameters Parameter list matching format specifiers
     */
    public static void notify(String message, Object... parameters) {
        DriverStation.reportWarning(String.format(message, parameters), false);
        Logger.consoleLog(message, parameters);
        warning.setString(String.format(message, parameters));
    }
}