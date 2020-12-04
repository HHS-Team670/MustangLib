/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.mustanglib.subsystems.drivebase;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.Logger;
import frc.team670.mustanglib.utils.MustangController;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.commands.drive.teleop.XboxRocketLeague.XboxRocketLeagueDrive;
import frc.team670.mustanglib.constants.RobotConstants;
import frc.team670.mustanglib.dataCollection.sensors.NavX;

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
  private NavX navXMicro;
  private DifferentialDriveOdometry m_odometry;

  /**
   * 
   * @param leftMotors Array of left side drivebase motor controllers, must have length > 0
   * @param rightMotors Array of right side drivebase motor controllers, must have length > 0
   * @param inverted Invert the motors (make what would have been the front the back)
   * @param rightSideInverted Invert the right motor outputs to counteract them being flipped comparatively with the left ones
   * @param deadband A minimum motor input to move the drivebase
   * @param safetyEnabled Safety Mode, enforces motor safety which turns off the motors if communication lost, other failures, etc.
   */
  public TankDriveBase(SpeedController[] leftMotors, SpeedController[] rightMotors, boolean inverted, boolean rightSideInverted, double deadband, boolean safetyEnabled, NavX navXMicro){
    setMotorControllers(leftMotors, rightMotors, inverted, rightSideInverted, deadband, safetyEnabled);
    // initialized NavX and sets Odometry
    this.navXMicro = navXMicro;
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()),
            new Pose2d(0, 0, new Rotation2d()));
    
  }

  /**
   * 
   * @param leftMotors Array of left side drivebase motor controllers, must have length > 0
   * @param rightMotors Array of right side drivebase motor controllers, must have length > 0
   */
  public TankDriveBase(SpeedController[] leftMotors, SpeedController[] rightMotors, NavX navXMicro) {
    this(leftMotors, rightMotors, true, true, 0.02, true, navXMicro);
  }

  /**
   * 
   * @param leftMotors Array of left side drivebase motor controllers, must have length > 0
   * @param rightMotors Array of right side drivebase motor controllers, must have length > 0
   * @param inverted Invert the motors (make what would have been the fron the back)
   * @param rightSideInverted Invert the right motor outputs to counteract them being flipped comparatively with the left ones
   */
  public TankDriveBase(SpeedController[] leftMotors, SpeedController[] rightMotors, boolean inverted, boolean rightSideInverted, NavX navXMicro) {
    this(leftMotors, rightMotors, inverted, rightSideInverted, 0.02, true, navXMicro);
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
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, in range [-180, 180]
   */
  public double getHeading() {
    return Math.IEEEremainder(navXMicro.getAngle(), 360) * (RobotConstants.kNavXReversed ? -1. : 1.);
  }

   /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose, CANEncoder left1Encoder, CANEncoder right1Encoder) {
    zeroHeading();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    CANError lE = left1Encoder.setPosition(0);
    CANError rE = right1Encoder.setPosition(0);
    Logger.consoleLog("Encoder return value %s %s", lE, rE);
    Logger.consoleLog("Encoder positions %s %s", left1Encoder.getPosition(), right1Encoder.getPosition());
    int counter = 0;
    while((left1Encoder.getPosition() != 0 || right1Encoder.getPosition() != 0) && counter <30){
      lE = left1Encoder.setPosition(0);
      rE = right1Encoder.setPosition(0);
      counter++;
    }
    Logger.consoleLog("Encoder return value %s %s", lE, rE);
    Logger.consoleLog("Encoder positions %s %s", left1Encoder.getPosition(), right1Encoder.getPosition());
    Logger.consoleLog("Drivebase pose reset %s", pose);
    Logger.consoleLog("Drivebase get position after reset %s %s", left1Encoder.getPosition(), right1Encoder.getPosition());
  }

    /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    navXMicro.reset();
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
