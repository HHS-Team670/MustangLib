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
import frc.team670.mustanglib.utils.JoystickUtils;
import frc.team670.mustanglib.utils.MustangController;

/**
 * Add your docs here.
 */
public class XboxCurvatureDrive extends CommandBase implements MustangCommand {

    private DriveBase driveBase;
    private MustangController controller;
    Joystick joystick;

    /**
     * Add your docs here.
     */
    public XboxCurvatureDrive(DriveBase driveBase, MustangController controller) {
        super();
        this.driveBase = driveBase;
        this.controller = controller;
        addRequirements(driveBase);
    }

    // Called once when the command executes
    @Override
    public void execute() {
        // Runs Curvature Drive with the left Joystick for steering and the right
        // joystick for throttle (smoothed by squaring input). QuickTurn is bound to
        // Right Bumper
        driveBase.curvatureDrive(-1 * JoystickUtils.smoothInput(controller.getRightStickY()),
                controller.getLeftStickX(),  controller.getRightBumper());
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        Map<MustangSubsystemBase, HealthState> healthRequirements = new HashMap<MustangSubsystemBase, HealthState>();
        healthRequirements.put(driveBase, HealthState.YELLOW);
        return healthRequirements;
    }

}