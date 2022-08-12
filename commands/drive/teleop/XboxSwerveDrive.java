package frc.team670.mustanglib.commands.drive.teleop;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.subsystems.SwerveDriveBase;


public class XboxSwerveDrive extends CommandBase implements MustangCommand {
    private final SwerveDriveBase driveBase;

    private MustangController controller;

    public XboxSwerveDrive(SwerveDriveBase swerveDriveBase, MustangController controller) {
        this.driveBase = swerveDriveBase;
        this.controller = controller;

        addRequirements(driveBase);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        double angle = controller.getRightX(); //m_rotationSupplier.getAsDouble();
        double xPos = controller.getLeftX(); //m_translationXSupplier.getAsDouble();
        double yPos =  controller.getLeftY(); //m_translationYSupplier.getAsDouble();
        
        driveBase.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        xPos,
                        yPos,
                        angle,
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
}
