package frc.team670.mustanglib.commands.drive.teleop.swerve;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;
import frc.team670.mustanglib.utils.MustangController;


public class XboxSwerveDrive extends Command implements MustangCommand {
    private final SwerveDrive driveBase;
   
    private MustangController controller;

    // private Rotation2d desiredHeading = null;
    private double MAX_VELOCITY, MAX_ANGULAR_VELOCITY;

    public XboxSwerveDrive(SwerveDrive swerveDriveBase, MustangController controller) {
        this.driveBase = swerveDriveBase;
        this.controller = controller;
      

        MAX_VELOCITY = swerveDriveBase.getMaxVelocityMetersPerSecond();
        MAX_ANGULAR_VELOCITY = swerveDriveBase.getMaxAngularVelocityMetersPerSecond();
        
        addRequirements(driveBase);
    }

    @Override
    public void execute() {
        // clear desired heading if at the heading or joystick touched
        if (driveBase.getDesiredHeading() != null) {
            if (modifyAxis(-controller.getRightX()) != 0)
                driveBase.setmDesiredHeading(null);
        }
        double xVel = MAX_VELOCITY * modifyAxis(-controller.getLeftY());
        double yVel = MAX_VELOCITY * modifyAxis(-controller.getLeftX());
        double thetaVel;
        thetaVel = MAX_ANGULAR_VELOCITY * modifyAxis(-controller.getRightX());

        driveBase.drive(xVel, yVel, thetaVel);

    }


    @Override
    public void end(boolean interrupted) {
        driveBase.drive(0.0, 0.0, 0.0);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        Map<MustangSubsystemBase, HealthState> healthRequirements =
                new HashMap<MustangSubsystemBase, HealthState>();
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
        return value;
    }




    public class SetDesiredHeading extends InstantCommand implements MustangCommand {
        Rotation2d desiredHeading;

        public SetDesiredHeading(Rotation2d desiredHeading) {
            this.desiredHeading = desiredHeading;
        }

        @Override
        public void initialize() {
            driveBase.setmDesiredHeading(desiredHeading);
        }

        @Override
        public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
            // TODO Auto-generated method stub
            return null;
        }

    }
}