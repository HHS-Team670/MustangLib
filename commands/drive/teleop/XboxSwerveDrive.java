package frc.team670.mustanglib.commands.drive.teleop;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;


public class XboxSwerveDrive extends CommandBase implements MustangCommand {
    private final SwerveDrive driveBase;

    private MustangController controller;

    private double MAX_VELOCITY, MAX_ANGULAR_VELOCITY;

    public XboxSwerveDrive(SwerveDrive swerveDriveBase, MustangController controller, double maxVelocity, double maxAngularVelocity) {
        this.driveBase = swerveDriveBase;
        this.controller = controller;

        MAX_VELOCITY = maxVelocity;
        MAX_ANGULAR_VELOCITY = maxAngularVelocity;

        addRequirements(driveBase);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        double angle = modifyAxis(-controller.getRightX()); //m_rotationSupplier.getAsDouble();
        double xPos = modifyAxis(-controller.getLeftY()); //m_translationXSupplier.getAsDouble();
        double yPos =  modifyAxis(-controller.getLeftX()); //m_translationYSupplier.getAsDouble();
        
        driveBase.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        xPos * MAX_VELOCITY,
                        yPos * MAX_VELOCITY,
                        angle * MAX_ANGULAR_VELOCITY,
                        driveBase.getGyroscopeRotation()
                )
        );
        
    }

    @Override
    public void end(boolean interrupted) {
        driveBase.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        Map<MustangSubsystemBase, HealthState> healthRequirements = new HashMap<MustangSubsystemBase, HealthState>();
        healthRequirements.put(driveBase, HealthState.YELLOW);
        return healthRequirements;
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
          if (value > 0.0) {
            return (value - deadband) / (1.0 - deadband);
          } else {
            return (value + deadband) / (1.0 - deadband);
          }
        } else {
          return 0.0;
        }
      }
    
      private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.05);
    
        // Square the axis
        //value = Math.copySign(value, value);
    
        return value;
      }
}
