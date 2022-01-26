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
import frc.team670.mustanglib.subsystems.drivebase.DriveBase;

/**
 * Add your docs here.
 */
public class WheelCurvatureDrive extends CommandBase implements MustangCommand {

    private DriveBase driveBase;

    private Joystick leftJoystick, rightJoystick;

    /**
     * Add your docs here.
     */
    public WheelCurvatureDrive(DriveBase driveBase, Joystick leftJoystick, Joystick rightJoystick) {
        super();
        this.driveBase = driveBase;
        this.leftJoystick = leftJoystick;
        this.rightJoystick = rightJoystick;
        addRequirements(driveBase);
    }

    // Called once when the command executes
    @Override
    public void execute() {
        // Robot.driveBase.curvatureDrive(xSpeed, zRotation, isQuickTurn);
        driveBase.curvatureDrive(-1 * rightJoystick.getY(), leftJoystick.getX(),
                false);
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        Map<MustangSubsystemBase, HealthState> healthRequirements = new HashMap<MustangSubsystemBase, HealthState>();
        healthRequirements.put(driveBase, HealthState.YELLOW);
        return healthRequirements;
    }

}