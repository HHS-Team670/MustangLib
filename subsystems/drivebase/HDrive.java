/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.mustanglib.subsystems.drivebase;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig.Motor_Type;


/**
 * 
 * Represents a h drive base using the WPIlib DifferentialDrive class. 
 * 
 * @author lakshbhambhani, armaan, aditi
 */
public abstract class HDrive extends TankDrive {

  
  private MotorController centerDrive;
  public static record Config (double kDriveBaseTrackWidth, double kDriveBaseGearRatio, double kDriveBaseWheelDiameter, 
  double kTrackWidthMeters, int kLeftLeaderMotorID, int kLeftFollowerMotorID, int kRightLeaderMotorID,
  int kRightFollowerMotorID,int kCenterMotorID, double deadband , boolean inverted) {
  }

    public HDrive(Config kConfig){
      super(new TankDrive.Config(kConfig.kDriveBaseTrackWidth,kConfig.kDriveBaseGearRatio,kConfig.kDriveBaseWheelDiameter,kConfig.kTrackWidthMeters,kConfig.kLeftLeaderMotorID, kConfig.kLeftFollowerMotorID, kConfig.kRightLeaderMotorID, kConfig.kRightFollowerMotorID, kConfig.deadband,kConfig.inverted ));
      centerDrive=SparkMAXFactory.buildFactorySparkMAX(kConfig.kCenterMotorID, Motor_Type.NEO);
    }
  

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
