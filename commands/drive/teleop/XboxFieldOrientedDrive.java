package frc.team670.mustanglib.commands.drive.teleop;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.dataCollection.sensors.NavX;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.drivebase.TankDrive;
import frc.team670.mustanglib.utils.MustangController;

/**
 *  @author Rathik, Ethan Chang, Aaditya R, Akshat Adzule, Benjamin
 * Note: This command is built for an omni/h-drive but h-drive functionality was removed and the command has not been updated since. Use caution
 */
public class XboxFieldOrientedDrive extends CommandBase implements MustangCommand {

    private MustangController m_controller = new MustangController(0);
    private TankDrive driveBase;
    private NavX navX;
    public final double maxSpeed = 0.5;
    private Map<MustangSubsystemBase, HealthState> healthRequirements = new HashMap<MustangSubsystemBase, HealthState>();


    public XboxFieldOrientedDrive(TankDrive driveBase, NavX navX, MustangController driverController) {
        this.driveBase = driveBase;
        this.navX = navX;
        this.m_controller = driverController;
        addRequirements(driveBase);
        healthRequirements.put(driveBase, HealthState.YELLOW);

    }

    @Override
    public void execute() {
        
        // double[] speeds = FieldOrientedDriveOmni.getComponentSpeeds(xSpeed, ySpeed, headingAngle);

        // driveBase.setCenterDrive(speeds[1]);
        // driveBase.curvatureDrive(speeds[0], 0, driveBase.isQuickTurnPressed());

        // get x and y components of joystick push
        double xSpeed = m_controller.getLeftStickX();
        double ySpeed = m_controller.getLeftStickY();   

        // get angle formed by field and robot heading
        double navXAngle = navX.getYawDouble(); 

        // twist from right joystick
        double zRotation = m_controller.getRightStickX();
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
        return healthRequirements;
    }

}
