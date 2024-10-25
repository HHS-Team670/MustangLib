package frc.team670.mustanglib.swervelib.ctre;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;
import frc.team670.mustanglib.RobotBase;

public final class CtreUtils {
  private CtreUtils() {}



  public static void checkCtreError(StatusCode statusCode, String message) {
    if (RobotBase.isReal() && statusCode != StatusCode.OK) {
      DriverStation.reportError(String.format("%s: %s", message, statusCode.toString()), false);
    }
  }
}
