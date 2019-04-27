/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * Add your docs here.
 */
public abstract class DriveBase extends Subsystem {

  protected DifferentialDrive drive; 

  public DriveBase(SpeedControllerGroup left, SpeedControllerGroup right){
    drive = new DifferentialDrive(left, right);
  }

  /**
   * Sets the Ramp Rate.
   * @param rampRate The time in seconds to go from zero to full speed.
   */
  public abstract void setRampRate(double rampRate);


  /**
   * Sets the drive base to drive. -1 is reverse, 1 is forward
   * @param left The left power [-1, 1]
   * @param right The right power [-1, 1]
   */
  public void tankDrive(double left, double right) {
    drive.tankDrive(left, right);
  }

}
