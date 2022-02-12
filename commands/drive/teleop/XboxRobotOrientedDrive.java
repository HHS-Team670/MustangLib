package frc.team670.mustanglib.commands.drive.teleop;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.dataCollection.sensors.NavX;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.robot.subsystems.DriveBase;

/**
 *  @author Rathik, Ethan Chang, Aaditya R, Akshat Adzule, Benjamin
 */
public class XboxRobotOrientedDrive extends CommandBase implements MustangCommand {

    private MustangController xbox = new MustangController(0);
    private DriveBase driveBase;
    public final double maxSpeed = 0.5;

    public XboxRobotOrientedDrive(DriveBase driveBase, MustangController xbox) {
        this.driveBase = driveBase;
        this.xbox = xbox;
        addRequirements(driveBase);
    }

    @Override
    public void execute() {
        
        // get x and y components of joystick push
        double xSpeed = xbox.getLeftStickX();
        double ySpeed = xbox.getLeftStickY();   

        // twist from right joystick
        double zRotation = xbox.getRightStickX();
        driveBase.setCenterDrive(xSpeed);
        driveBase.curvatureDrive(-ySpeed, zRotation, true);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        Map<MustangSubsystemBase, HealthState> healthRequirements = new HashMap<MustangSubsystemBase, HealthState>();
        healthRequirements.put(driveBase, HealthState.YELLOW);
        return healthRequirements;
    }    
}
