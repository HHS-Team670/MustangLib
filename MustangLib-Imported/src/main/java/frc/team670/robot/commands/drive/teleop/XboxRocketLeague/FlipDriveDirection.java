/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.drive.teleop.XboxRocketLeague;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.team670.robot.RobotContainer;
import frc.team670.robot.commands.MustangCommand;
import frc.team670.robot.subsystems.MustangSubsystemBase;
import frc.team670.robot.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.robot.utils.Logger;

/**
 * Flips the direction of the drive: forward or reversed.
 */
public class FlipDriveDirection extends InstantCommand implements MustangCommand {

  public FlipDriveDirection() {
    super();
  }

  // Called once when the command executes
  public void initialize() {
    boolean isReversed = XboxRocketLeagueDrive.isDriveReversed();
    // if (!isReversed) {
    // Robot.leds.setReverseData(true);
    // } else {
    // Robot.leds.setForwardData(true);
    // }
    XboxRocketLeagueDrive.setDriveReversed(!isReversed);
    // RobotContainer.rumbleDriverController();
    Logger.consoleLog("Flipped Drive: %s", (!isReversed));
  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    // TODO Auto-generated method stub
    return null;
  }

}
