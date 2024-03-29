
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.mustanglib.commands.drive.teleop.tank.XboxRocketLeague;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.drivebase.TankDrive;
import frc.team670.mustanglib.utils.MustangController;

/**
 * Tank Drive command
 * Note: this is for tank drive,
 * Right stick up and down to move the right side of the robot
 * Left stick up and down to move the left side of the robot
 */

public class XboxTankDrive extends Command implements MustangCommand {
    
    private TankDrive driveBase;
    private MustangController controller;
    private Map<MustangSubsystemBase, HealthState> healthRequirements = new HashMap<MustangSubsystemBase, HealthState>();


    /**
     * Contructing a new tank drive
     */
    public XboxTankDrive(TankDrive driveBase, MustangController controller) {
        super();
        this.driveBase = driveBase;
        this.controller = controller;
        addRequirements(driveBase);
        healthRequirements.put(driveBase, HealthState.YELLOW);

    }

    // Called once when the command executes
    @Override
    public void execute() {

        driveBase.tankDrive(-1 * controller.getLeftStickY(),
                -1 * controller.getRightStickY());
    }


    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthRequirements;
    }



}