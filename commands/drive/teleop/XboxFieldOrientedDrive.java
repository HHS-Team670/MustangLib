package frc.team670.mustanglib.commands.drive.teleop;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.dataCollection.sensors.NavX;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.drivebase.DriveBase;
import frc.team670.mustanglib.utils.MustangController;

/**
 *  @author Rathik, Ethan Chang, Aaditya R, Akshat Adzule, Benjamin
 */
public class XboxFieldOrientedDrive extends CommandBase implements MustangCommand {

    private MustangController xbox = new MustangController(0);
    private DriveBase driveBase;
    private NavX navX;
    public final double maxSpeed = 0.5;

    public XboxFieldOrientedDrive(DriveBase driveBase, NavX navX, MustangController xbox) {
        this.driveBase = driveBase;
        this.navX = navX;
        this.xbox = xbox;
        addRequirements(driveBase);
    }

    @Override
    public void execute() {
        
        // double[] speeds = FieldOrientedDriveOmni.getComponentSpeeds(xSpeed, ySpeed, headingAngle);

        // driveBase.setCenterDrive(speeds[1]);
        // driveBase.curvatureDrive(speeds[0], 0, driveBase.isQuickTurnPressed());

        // get x and y components of joystick push
        double xSpeed = xbox.getLeftStickX();
        double ySpeed = xbox.getLeftStickY();   

        // get angle formed by field and robot heading
        double navXAngle = navX.getYawDouble(); 

        // twist from right joystick
        double zRotation = xbox.getRightStickX();
        double[] speeds = getComponentSpeeds(xSpeed, ySpeed, navXAngle);

        driveBase.curvatureDrive(speeds[0], zRotation, true);
    }

    private double[] getComponentSpeeds(double xSpeed, double ySpeed, double navXAngle) {
    
    
        // convert to radians
        double angle_radians = navXAngle * Math.PI / 180;

        //frwrd y, strafe x
        double frwd = -xSpeed * Math.sin(angle_radians) + ySpeed * Math.cos(angle_radians);
        double strafe = xSpeed * Math.cos(angle_radians) + ySpeed * Math.sin(angle_radians);

        return new double[] {-frwd, strafe};
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        Map<MustangSubsystemBase, HealthState> healthRequirements = new HashMap<MustangSubsystemBase, HealthState>();
        healthRequirements.put(driveBase, HealthState.YELLOW);
        return healthRequirements;
    }    
}
