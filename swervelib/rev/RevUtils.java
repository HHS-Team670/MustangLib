package frc.team670.mustanglib.swervelib.rev;

import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DriverStation;
import frc.team670.mustanglib.RobotBase;

public final class RevUtils {
  private RevUtils() {}

  public static void checkNeoError(REVLibError error, String message) {
    if (RobotBase.isReal() && error != REVLibError.kOk) {
      DriverStation.reportError(String.format("%s: %s", message, error.toString()), false);
    }
  }
}
