/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.mustanglib.commands.drive.teleop.tank.XboxRocketLeague;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.drivebase.TankDrive;

/**
 *  Wheel curvature drive
 * Note: this is for tank drive
 */
public class WheelCurvatureDrive extends Command implements MustangCommand {

    private TankDrive driveBase;

    private Joystick leftJoystick, rightJoystick;

    private Map<MustangSubsystemBase, HealthState> healthRequirements = new HashMap<MustangSubsystemBase, HealthState>();


    /**
     * Constructor for wheel curvature drive
     */
    public WheelCurvatureDrive(TankDrive driveBase, Joystick leftJoystick, Joystick rightJoystick) {
        super();
        this.driveBase = driveBase;
        this.leftJoystick = leftJoystick;
        this.rightJoystick = rightJoystick;
        addRequirements(driveBase);
        healthRequirements.put(driveBase, HealthState.YELLOW);

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
        return healthRequirements;
    }

}