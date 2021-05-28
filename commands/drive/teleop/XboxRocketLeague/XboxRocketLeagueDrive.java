/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.mustanglib.commands.drive.teleop.XboxRocketLeague;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.drivebase.TankDriveBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.utils.functions.JoystickUtils;

/**
 * Drives the Robot using Xbox controls like the game Rocket League. Triggers
 * control speed, stick is for steering.
 * 
 * @author lakshbhambhani
 */
public class XboxRocketLeagueDrive extends CommandBase implements MustangCommand {

  private static boolean isReversed;

  private TankDriveBase driveBase;
  private MustangController controller;

  public XboxRocketLeagueDrive(TankDriveBase driveBase, MustangController controller) {
    isReversed = false;
    addRequirements(driveBase);
    this.driveBase = driveBase;
    this.controller = controller;
  }

  // Called once when the command executes
  @Override
  public void execute() {
    // Sets the speed to the reading given by the trigger axes on the controller.
    // Left is positive, but we multiply
    // by -1 to reverse that because we want right trigger to correspond to forward.
    double speed = -1 * (controller.getLeftTriggerAxis()
        - controller.getRightTriggerAxis());
    double steer = controller.getLeftStickX();

    // Decides whether or not to smooth the Steering and Trigger. Smoothing helps
    // reduce jerkiness when driving.
    // tankDrive actually does this for us automatically, so npo need to do it
    // ourselves
    steer = JoystickUtils.smoothInput(steer);
    speed = JoystickUtils.smoothInput(speed);

    if (XboxRocketLeagueDrive.isDriveReversed()) {
      steer *= -1;
      speed *= -1;
    }

    if (driveBase.isQuickTurnPressed()) {

      if (speed < -0.0001) {
        if (!XboxRocketLeagueDrive.isDriveReversed()) {
          driveBase.curvatureDrive(speed, -1 * steer, driveBase.isQuickTurnPressed());
        } else {
          driveBase.curvatureDrive(speed, -1 * steer, driveBase.isQuickTurnPressed());
        }
      } else if (speed > 0.0001) {
        if (!XboxRocketLeagueDrive.isDriveReversed()) {
          driveBase.curvatureDrive(speed, steer, driveBase.isQuickTurnPressed());
        } else {
          driveBase.curvatureDrive(speed, steer, driveBase.isQuickTurnPressed());
        }
      } else {
        if (!XboxRocketLeagueDrive.isDriveReversed()) {
          driveBase.curvatureDrive(speed, steer, driveBase.isQuickTurnPressed());
        } else {
          driveBase.curvatureDrive(speed, -1 * steer, driveBase.isQuickTurnPressed());
        }
      }
    } else {
      if (speed < -0.0001) {
        driveBase.curvatureDrive(speed, -1 * steer, driveBase.isQuickTurnPressed());
      } else {
        driveBase.curvatureDrive(speed, steer, driveBase.isQuickTurnPressed());
      }
    }
  }

  public static boolean isDriveReversed() {
    return isReversed;
  }

  @Override
  public boolean isFinished(){
    return false;
  }

  public static void setDriveReversed(boolean reversed) {
    XboxRocketLeagueDrive.isReversed = reversed;
    SmartDashboard.putBoolean("drive-reversed", reversed);
  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    Map<MustangSubsystemBase, HealthState> healthRequirements = new HashMap<MustangSubsystemBase, HealthState>();
    healthRequirements.put(driveBase, HealthState.YELLOW);
    return healthRequirements;
  }

}