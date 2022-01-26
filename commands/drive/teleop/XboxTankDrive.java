
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.mustanglib.commands.drive.teleop;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.drivebase.DriveBase;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.MustangController;

/**
 * Add your docs here.
 */
public class XboxTankDrive extends CommandBase implements MustangCommand {

    private DriveBase driveBase;
    private MustangController controller;

    /**
     * Add your docs here.
     */
    public XboxTankDrive(DriveBase driveBase, MustangController controller) {
        super();
        addRequirements(driveBase);
        this.driveBase = driveBase;
        this.controller = controller;
    }

    // Called once when the command executes
    @Override
    public void execute() {

        driveBase.tankDrive(-1 * controller.getLeftStickY(),
                -1 * controller.getRightStickY());
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        Map<MustangSubsystemBase, HealthState> healthRequirements = new HashMap<MustangSubsystemBase, HealthState>();
        healthRequirements.put(driveBase, HealthState.YELLOW);
        return healthRequirements;
    }

}