/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.mustanglib.subsystems.drivebase;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;


/**
 * 
 * Represents a h drive base using the WPIlib DifferentialDrive class. 
 * 
 * @author lakshbhambhani, armaan, aditi
 */
public abstract class HDrive extends TankDrive {

  
  private MotorController centerDrive;
  
  /**
   * 
   * @param leftMotors Array of left side drivebase motor controllers, must have length greater than 0
   * @param rightMotors Array of right side drivebase motor controllers, must have length greater than 0
   * @param inverted Invert the motors (make what would have been the front the back)
   * @param rightSideInverted Invert the right motor outputs to counteract them being flipped comparatively with the left ones
   * @param deadband A minimum motor input to move the drivebase
   * @param safetyEnabled Safety Mode, enforces motor safety which turns off the motors if communication lost, other failures, etc.
   */
  public HDrive(MotorController leftMotor, MotorController rightMotor, MotorController centerDrive, boolean inverted, boolean rightSideInverted, double deadband, boolean safetyEnabled){
    setMotorControllers(leftMotor, rightMotor, centerDrive, inverted, rightSideInverted, deadband, safetyEnabled);
    
  }

  /**
   * 
   * @param leftMotor Leader of the left motors
   * @param rightMotor Leader of the right motors
   */
  public HDrive(MotorController leftMotor, MotorController rightMotor, MotorController centerDrive) {
    this(leftMotor, rightMotor, centerDrive, true, true, 0.02, true);
  }

  /**
   * 
   * @param leftMotor Leader of the left motors
   * @param rightMotor Leader of the right motors
   * @param inverted Invert the motors (make what would have been the fron the back)
   * @param rightSideInverted Invert the right motor outputs to counteract them being flipped comparatively with the left ones
   */
  public HDrive(MotorController leftMotor, MotorController rightMotor, MotorController centerDrive, boolean inverted, boolean rightSideInverted) {
    this(leftMotor, rightMotor, centerDrive, inverted, rightSideInverted, 0.02, true);
  }

  /**
   * Usethis constructor as the super() in a sublcass, then call setMotorControllers if you need to run setup on Motor Controllers
   */
  public HDrive() {}

  /**
   * This method is called by the constructor. Much of the time setup needs to be performed on motors, so perform the setup in a subclass, then call this method.
   * @param leftMotor Leader of the left motors
   * @param rightMotor Leader of the right motors
   * @param inverted Invert the motors (make what would have been the front the back)
   * @param rightSideInverted Invert the right motor outputs to counteract them being flipped comparatively with the left ones
   * @param deadband A minimum motor input to move the drivebase
   * @param safetyEnabled Safety Mode, enforces motor safety which turns off the motors if communication lost, other failures, etc.
   */
  protected void setMotorControllers(MotorController leftMotor, MotorController rightMotor, MotorController centerDrive, boolean inverted, boolean rightSideInverted, double deadband, boolean safetyEnabled) {
    super.setMotorControllers(leftMotor, rightMotor,inverted,rightSideInverted,deadband,safetyEnabled);
    this.centerDrive = centerDrive;
  }

  /**
   * This method is called by the constructor or in a subclass if motor setup needs to be performed. Much of the time setup needs to be performed on motors, so perform the setup in a subclass, then call this method.
   * @param leftMotor Leader of the left motors 
   * @param rightMotor Array of right side drivebase motor controllers, must have length greater than 0
   */
  protected void setMotorControllers(MotorController leftMotor, MotorController rightMotor, MotorController centerDrive) {
    setMotorControllers(leftMotor, rightMotor, centerDrive, true, true, 0.02, true);
  }

  /**
   * This method is called by the constructor. Much of the time setup needs to be performed on motors, so perform the setup in a subclass, then call this method.
   * @param leftMotor Leader of the left motors
   * @param rightMotor Leader of the right motors
   * @param inverted Invert the motors (make what would have been the front the back)
   * @param rightSideInverted Invert the right motor outputs to counteract them being flipped comparatively with the left ones
   */
  public void setMotorControllers(MotorController leftMotor, MotorController rightMotor, MotorController centerDrive, boolean inverted, boolean rightSideInverted) {
    setMotorControllers(leftMotor, rightMotor, centerDrive, inverted, rightSideInverted, 0.02, true);
  }

  public void strafe(double speed) {
    centerDrive.set(speed);
  }

  /**
   * Checks the health for driveBase. RED if all motors are dead, GREEN if all
   * motors are alive, YELLOW if a motor is disconnected
   * @param isLeft1Error boolean for left1 motor being errored
   * @param isLeft2Error boolean for left2 motor being errored
   * @param isRight1Error boolean for right1 motor being errored
   * @param isRight2Error boolean for right2 motor being errored
   * @param isCenterError boolean for center module motor being errored
   */
  public HealthState checkHealth(boolean isLeft1Error, boolean isLeft2Error, boolean isRight1Error, boolean isRight2Error, boolean isCenterError) {
    HealthState state = HealthState.GREEN;

    if (!isLeft1Error && !isLeft2Error && !isRight1Error && !isRight2Error && !isCenterError) {
      state = HealthState.GREEN;
    } else if (isLeft1Error && isLeft2Error || isRight1Error && isRight2Error) {
      state = HealthState.RED;
    } else {
      state = HealthState.YELLOW;
    }
    return state;
  }
}
