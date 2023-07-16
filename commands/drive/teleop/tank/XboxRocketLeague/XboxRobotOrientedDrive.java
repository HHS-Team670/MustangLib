// package frc.team670.mustanglib.commands.drive.teleop.tank.XboxRocketLeague;

// import java.util.HashMap;
// import java.util.Map;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.team670.mustanglib.commands.MustangCommand;
// import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
// import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
// import frc.team670.mustanglib.subsystems.drivebase.TankDrive;
// import frc.team670.mustanglib.utils.JoystickUtils;
// import frc.team670.mustanglib.utils.MustangController;

// /**
//  *  @author Rathik, Ethan Chang, Aaditya R, Akshat Adzule, Benjamin, armaan g, aditi k
//  * Note: this is for tank drive
//  */
// public class XboxRobotOrientedDrive extends CommandBase implements MustangCommand {

//     private MustangController m_controller = new MustangController(0);
//     private TankDrive driveBase;
//     public final double maxSpeed = 0.5;
//     private Map<MustangSubsystemBase, HealthState> healthRequirements = new HashMap<MustangSubsystemBase, HealthState>();


//     public XboxRobotOrientedDrive(TankDrive driveBase, MustangController driverController) {
//         this.driveBase = driveBase;
//         this.m_controller = driverController;
//         addRequirements(driveBase);
//         healthRequirements.put(driveBase, HealthState.YELLOW);

//     }

//     @Override
//     public void execute() {
        
//         // get x and y components of joystick push
//         double ySpeed = JoystickUtils.smoothInput(m_controller.getLeftStickY());

//         // twist from right joystick
//         double zRotation = -JoystickUtils.smoothInput(m_controller.getRightStickX());
//         driveBase.curvatureDrive(-ySpeed, zRotation, true);
//     }

//     @Override
//     public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
//         return healthRequirements;
//     }  
// }
