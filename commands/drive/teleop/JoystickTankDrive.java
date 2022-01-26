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
import frc.team670.mustanglib.subsystems.drivebase.DriveBase;

public class JoystickTankDrive extends CommandBase implements MustangCommand {

    private DriveBase driveBase;
    private Joystick leftJoystick, rightJoystick;

    public JoystickTankDrive(DriveBase driveBase, Joystick leftJoystick, Joystick rightJoystick) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        super();
        this.driveBase = driveBase;
        this.leftJoystick = leftJoystick;
        this.rightJoystick = rightJoystick;
        addRequirements(driveBase);
    }

    // Called just before this Command runs the first time
    @Override
    public void execute() {
        // TODO use the drive calculator to adjust outputs
        driveBase.tankDrive(-1 * leftJoystick.getY(), -1 * rightJoystick.getY());
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        Map<MustangSubsystemBase, HealthState> healthRequirements = new HashMap<MustangSubsystemBase, HealthState>();
        healthRequirements.put(driveBase, HealthState.YELLOW);
        return healthRequirements;
    }

}
