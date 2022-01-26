/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.mustanglib.commands.drive.straight;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.drivebase.DriveBase;

public class TimedDrive extends WaitCommand implements MustangCommand {

  private DriveBase driveBase;
  private double power;

  public TimedDrive(double seconds, double power, DriveBase driveBase) {
    super(seconds);
    this.power = power;
    this.driveBase = driveBase;
    addRequirements(driveBase);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    driveBase.tankDrive(power, power);
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean isInteruppted) {
    driveBase.stop();
  }

  @Override
  public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
    Map<MustangSubsystemBase, HealthState> healthRequirements = new HashMap<MustangSubsystemBase, HealthState>();
    healthRequirements.put(driveBase, HealthState.YELLOW);
    return healthRequirements;
  }
}