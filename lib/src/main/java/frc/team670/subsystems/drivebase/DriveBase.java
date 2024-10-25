/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.mustanglib.subsystems.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;

/*
 * 
 * Represents a drivebase 
 * 
 * @author shaylandias, lakshbhambhani, armaan, aditi
 */
public abstract class DriveBase extends MustangSubsystemBase {

  public abstract void initBrakeMode();

  public abstract void initCoastMode();

  public abstract void toggleIdleMode();

  public abstract void zeroHeading();
    
  //following for Mustang Command to get ramsete command
  public abstract Pose2d getPose();

  public abstract void resetOdometry(Pose2d pose);
      /**
     * Stops the motors on the drive base (sets them to 0).
     */
  public  abstract void stop();

}
