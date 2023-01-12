/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.mustanglib.commands.vision;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.subsystems.VisionSubsystemBase;

/**
 * Set vision leds to turned on or off
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
    Logger.consoleLog("LEds turned to: %s", turnOn);
    vision.switchLEDS(turnOn, true);
  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    return null;
  }

}
