// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.team670.mustanglib.commands.drive.teleop.tank.XboxRocketLeague;

// import java.util.HashMap;
// import java.util.Map;

// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.team670.mustanglib.commands.MustangCommand;
// import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
// import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
// import frc.team670.mustanglib.subsystems.drivebase.TankDrive;
// import frc.team670.mustanglib.utils.JoystickUtils;
// import frc.team670.mustanglib.utils.MustangController;

// /**
//  * 
//  * Note: for tank drive
//  * Also known as Cheesy Drive
//  * The "curvature" naming comes from the algorithm's change to the way that the "turn" joystick input is processed. When you a driving forward or backwards and turning at the same time, the turn input affects the curvature of the movement rather than adding/subtracting linearly from the wheel speeds. The turn output is a sum of the throttle and turn inputs, meaning that the robot will turn faster when it's moving forward at a higher speed. Again, the goal here is more control at low speeds. Note that the turn input is not changed when the robot is turning in place, this just affects the way the robot drives in an "S" shaped path.
//  * Cheesy Drive also applies some non-linearity to the joystick input so that there is more control at the low speeds. Larger changes in joystick inputs result in smaller changes in real speed when driving slow, but when the joystick is pushed to a high speed, you jump up to full speed faster.
//  * Third, that turn input is affected by a negative inertia accumulator. Most robots have a fair bit of turning inertia, which can make it easy to accidentally overshoot a turn. The negative inertia accumulator acts almost like a reverse integral controller  the longer the robot has been turning (fast) for, the slower the robot will turn.
//  */
// public class XboxCurvatureDrive extends CommandBase implements MustangCommand {

//     private TankDrive driveBase;
//     private MustangController controller;
//     Joystick joystick;
//     private Map<MustangSubsystemBase, HealthState> healthRequirements = new HashMap<MustangSubsystemBase, HealthState>();


//     /**
//      * Constructs a new XboxCurvatureDrive.
//      * @param driveBase the drrivebase to be drivnr
//      * @param controller the controllers to be used
//      */
//     public XboxCurvatureDrive(TankDrive driveBase, MustangController controller) {
//         super();
//         this.driveBase = driveBase;
//         this.controller = controller;
//         addRequirements(driveBase);
//         healthRequirements.put(driveBase, HealthState.YELLOW);

//     }

//     // Called once when the command executes
//     @Override
//     public void execute() {
//         // Runs Curvature Drive with the left Joystick for steering and the right
//         // joystick for throttle (smoothed by squaring input). QuickTurn is bound to
//         // Right Bumper
//         driveBase.curvatureDrive(-1 * JoystickUtils.smoothInput(controller.getRightStickY()),
//                 controller.getLeftStickX(),  controller.getRightBumper());
//     }

//     @Override
//     public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
//         return healthRequirements;
//     }


// }