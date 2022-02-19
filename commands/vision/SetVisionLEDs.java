/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.mustanglib.commands.vision;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.team670.robot.RobotContainer;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.VisionSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.robot.subsystems.Vision;

/**
 * Flips the direction of the drive: forward or reversed.
 */
public class SetVisionLEDs extends InstantCommand implements MustangCommand {

  boolean turnOn;
  VisionSubsystemBase vision;

  public SetVisionLEDs(boolean turnOn, VisionSubsystemBase vision) {
    super();
    this.turnOn = turnOn;
    this.vision = vision;
  }

  // Called once when the command executes
  public void initialize() {
     vision.switchLEDS(turnOn);
  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    // TODO Auto-generated method stub
    return null;
  }

}
