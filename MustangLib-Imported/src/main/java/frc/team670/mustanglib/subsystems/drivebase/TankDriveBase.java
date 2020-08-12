/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.mustanglib.subsystems.drivebase;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.commands.drive.teleop.XboxRocketLeague.XboxRocketLeagueDrive;

/**
 * 
 * Represents a tank drive base using the WPIlib DifferentialDrive class. Defaults to using XboxRocketLeagueDrive.
 * This can be overriden using the setDefaultCommand() method
 * 
 * @author shaylandias, lakshbhambhani
 */
public abstract class TankDriveBase extends MustangSubsystemBase {

  private SpeedControllerGroup leftMotors, rightMotors;
  protected DifferentialDrive drive; 

  /**
   * 
   * @param leftMotors Array of left side drivebase motor controllers, must have length > 0
   * @param rightMotors Array of right side drivebase motor controllers, must have length > 0
   * @param inverted Invert the motors (make what would have been the front the back)
   * @param rightSideInverted Invert the right motor outputs to counteract them being flipped comparatively with the left ones
   * @param deadband A minimum motor input to move the drivebase
   * @param safetyEnabled Safety Mode, enforces motor safety which turns off the motors if communication lost, other failures, etc.
   */
  public TankDriveBase(SpeedController[] leftMotors, SpeedController[] rightMotors, boolean inverted, boolean rightSideInverted, double deadband, boolean safetyEnabled){
    setMotorControllers(leftMotors, rightMotors, inverted, rightSideInverted, deadband, safetyEnabled);
    
  }

  /**
   * 
   * @param leftMotors Array of left side drivebase motor controllers, must have length > 0
   * @param rightMotors Array of right side drivebase motor controllers, must have length > 0
   */
  public TankDriveBase(SpeedController[] leftMotors, SpeedController[] rightMotors) {
    this(leftMotors, rightMotors, true, true, 0.02, true);
  }

  /**
   * 
   * @param leftMotors Array of left side drivebase motor controllers, must have length > 0
   * @param rightMotors Array of right side drivebase motor controllers, must have length > 0
   * @param inverted Invert the motors (make what would have been the fron the back)
   * @param rightSideInverted Invert the right motor outputs to counteract them being flipped comparatively with the left ones
   */
  public TankDriveBase(SpeedController[] leftMotors, SpeedController[] rightMotors, boolean inverted, boolean rightSideInverted) {
    this(leftMotors, rightMotors, inverted, rightSideInverted, 0.02, true);
  }

  /**
   * Usethis constructor as the super() in a sublcass, then call setMotorControllers if you need to run setup on Motor Controllers
   */
  public TankDriveBase() {}

  /**
   * This method is called by the constructor. Much of the time setup needs to be performed on motors, so perform the setup in a subclass, then call this method.
   * @param leftMotors Array of left side drivebase motor controllers, must have length > 0
   * @param rightMotors Array of right side drivebase motor controllers, must have length > 0
   * @param inverted Invert the motors (make what would have been the front the back)
   * @param rightSideInverted Invert the right motor outputs to counteract them being flipped comparatively with the left ones
   * @param deadband A minimum motor input to move the drivebase
   * @param safetyEnabled Safety Mode, enforces motor safety which turns off the motors if communication lost, other failures, etc.
   */
  protected void setMotorControllers(SpeedController[] leftMotors, SpeedController[] rightMotors, boolean inverted, boolean rightSideInverted, double deadband, boolean safetyEnabled) {
    this.leftMotors = generateControllerGroup(leftMotors);
    this.rightMotors = generateControllerGroup(rightMotors);
    drive = new DifferentialDrive(this.leftMotors, this.rightMotors);
    drive.setRightSideInverted(rightSideInverted);
    drive.setDeadband(deadband);
    drive.setSafetyEnabled(safetyEnabled);
  }

  /**
   * This method is called by the constructor or in a subclass if motor setup needs to be performed. Much of the time setup needs to be performed on motors, so perform the setup in a subclass, then call this method.
   * @param leftMotors Array of left side drivebase motor controllers, must have length > 0
   * @param rightMotors Array of right side drivebase motor controllers, must have length > 0
   */
  protected void setMotorControllers(SpeedController[] leftMotors, SpeedController[] rightMotors) {
    setMotorControllers(leftMotors, rightMotors, true, true, 0.02, true);
  }

  /**
   * This method is called by the constructor. Much of the time setup needs to be performed on motors, so perform the setup in a subclass, then call this method.
   * @param leftMotors Array of left side drivebase motor controllers, must have length > 0
   * @param rightMotors Array of right side drivebase motor controllers, must have length > 0
   * @param inverted Invert the motors (make what would have been the front the back)
   * @param rightSideInverted Invert the right motor outputs to counteract them being flipped comparatively with the left ones
   */
  public void setMotorControllers(SpeedController[] leftMotors, SpeedController[] rightMotors, boolean inverted, boolean rightSideInverted) {
    setMotorControllers(leftMotors, rightMotors, inverted, rightSideInverted, 0.02, true);
  }


  private SpeedControllerGroup generateControllerGroup(SpeedController[] motors) {
    if(motors.length > 0) {
      SpeedControllerGroup group;
      if(motors.length > 1) {
        SpeedController[] otherMotors = new SpeedController[motors.length - 1];
        for(int i = 1; i < motors.length; i++) {
          otherMotors[i-1] = motors[i];
        }
        group = new SpeedControllerGroup(motors[0], otherMotors);
      } else {
        group = new SpeedControllerGroup(motors[0]);
      }
      return group;
    }
    return null;
  }

  /**
   * Sets the Ramp Rate.
   * @param rampRate The time in seconds to go from zero to full speed.
   */
  public abstract void setRampRate(double rampRate);

   /**
   * 
   * Drives the Robot using a tank drive configuration (two joysticks, or auton).
   * Squares inputs to linearize them.
   * 
   * @param leftSpeed  Speed for left side of drive base [-1, 1]. Automatically
   *                   squares this value to linearize it.
   * @param rightSpeed Speed for right side of drive base [-1, 1]. Automatically
   *                   squares this value to linearize it.
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    tankDrive(leftSpeed, rightSpeed, true);
  }
  /**
   * 
   * Drives the Robot using a tank drive configuration (two joysticks, or auton)
   * 
   * @param leftSpeed     Speed for left side of drive base [-1, 1]
   * @param rightSpeed    Speed for right side of drive base [-1, 1]
   * @param squaredInputs If true, decreases sensitivity at lower inputs
   */
  public void tankDrive(double leftSpeed, double rightSpeed, boolean squaredInputs) {
    drive.tankDrive(leftSpeed, rightSpeed, squaredInputs);
  }

  /**
   * 
   * Drives the Robot using a curvature drive configuration (wheel)
   * 
   * @param xSpeed      The forward throttle speed [-1, 1]
   * @param zRotation   The amount of rotation to turn [-1, 1] with positive being
   *                    right
   * @param isQuickTurn If true enables turning in place and running one side
   *                    backwards to turn faster
   */
  public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
    drive.curvatureDrive(xSpeed, zRotation, isQuickTurn);
  }

  /**
   * 
   * Drives the Robot using an arcade drive configuration (single joystick with
   * twist)
   * 
   * @param xSpeed      The forward throttle speed [-1, 1]
   * @param zRotation   The amount of rotation to turn [-1, 1] with positive being
   *                    right
   * @param isQuickTurn If true, decreases sensitivity at lower inputs
   */
  public void arcadeDrive(double xSpeed, double zRotation, boolean squaredInputs) {
    drive.arcadeDrive(xSpeed, zRotation, squaredInputs);
  }

  /**
   * 
   * Drives the Robot using an arcade drive configuration (single joystick with
   * twist). Squares inputs for smoothing.
   * 
   * @param xSpeed    The forward throttle speed [-1, 1]
   * @param zRotation The amount of rotation to turn [-1, 1] with positive being
   *                  right
   */
  public void arcadeDrive(double xSpeed, double zRotation) {
    arcadeDrive(xSpeed, zRotation, true);
  }

  /**
   * Stops the motors on the drive base (sets them to 0).
   */
  public void stop() {
    drive.stopMotor();
  }

  public abstract void initBrakeMode();

  public abstract void initCoastMode();

  /**
   * Sets the velocities of the left and right motors of the robot.
   * @param leftVel  Velocity for left motors in inches/sec
   * @param rightVel Velocity for right motors in inches/sec
   */
  public abstract void setVelocityControl(double leftVel, double rightVel);

  /**
   * Sets the PIDControllers setpoints for the left and right side motors to the
   * given positions in inches forward.
   * 
   * @param deltaLeft  The desired change in left position in inches
   * @param deltaRight The desired change in right position in inches
   */
  public abstract void setEncodersPositionControl(double deltaLeft, double deltaRight);

  public abstract double inchesToTicks(double inches);

  public abstract double ticksToInches(double ticks);

  public abstract double getLeftPositionTicks();

  public abstract double getRightPositionTicks();

  public double getLeftPositionInches() {
    return ticksToInches(getLeftPositionTicks());
  }

  public double getRightPositionInches() {
    return ticksToInches(getRightPositionTicks());
  }

  /**
   * @return Velocity of the left motors in ticks per second
   */
  public abstract double getLeftVelocityTicks();

  /**
   * @return Velocity of the right motors in ticks per second
   */
  public abstract double getRightVelocityTicks();

  /**
   * @return Velocity of the left motors in inches per second
   */
  public double getLeftVelocityInches() {
    return ticksToInches(getLeftVelocityTicks());
  }

  /**
   * @return Velocity of the right motors in inches per second
   */
  public double getRightVelocityInches() {
    return ticksToInches(getRightVelocityTicks());
  }

  /**
   * Used to initialized teleop command for the driveBase
   */
  public void initDefaultCommand() {
    MustangScheduler.getInstance().setDefaultCommand(this, new XboxRocketLeagueDrive(this, getMustangController()));
  }

  public abstract boolean isQuickTurnPressed();

  public abstract MustangController getMustangController();

}
