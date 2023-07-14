/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.mustanglib.commands.drive.teleop;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.drivebase.TankDrive;
/**
 * Note: this is for tank drive only
 */
public class JoystickTankDrive extends CommandBase implements MustangCommand {

    private TankDrive driveBase;
    private Joystick leftJoystick, rightJoystick;
    private Map<MustangSubsystemBase, HealthState> healthRequirements = new HashMap<MustangSubsystemBase, HealthState>();
    public JoystickTankDrive(TankDrive driveBase, Joystick leftJoystick, Joystick rightJoystick) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        super();
        this.driveBase = driveBase;
        this.leftJoystick = leftJoystick;
        this.rightJoystick = rightJoystick;
        addRequirements(driveBase);
        healthRequirements.put(driveBase, HealthState.YELLOW);

    }

    // Called just before this Command runs the first time
    @Override
    public void execute() {
        driveBase.tankDrive(-1 * leftJoystick.getY(), -1 * rightJoystick.getY());
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
                return healthRequirements;
    }

}
