/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
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
import frc.team670.mustanglib.subsystems.drivebase.TankDriveBase;

public class SingleJoystickDrive extends CommandBase implements MustangCommand {

    private TankDriveBase driveBase;
    private Joystick leftJoystick;

    public SingleJoystickDrive(TankDriveBase driveBase, Joystick leftJoystick) {
        super();
        this.driveBase = driveBase;
        this.leftJoystick = leftJoystick;
        addRequirements(driveBase);
    }

    // Called just before this Command runs the first time
    @Override
    public void execute() {
        driveBase.arcadeDrive(-1 * leftJoystick.getY(), -1 * leftJoystick.getTwist(), true);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        Map<MustangSubsystemBase, HealthState> healthRequirements = new HashMap<MustangSubsystemBase, HealthState>();
        healthRequirements.put(driveBase, HealthState.YELLOW);
        return healthRequirements;
    }

}
