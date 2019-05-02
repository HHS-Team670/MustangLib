package frc.team670.robot.subsystems.drivebase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team670.robot.commands.XboxRocketLeagueDrive;

public class SRXTankDrive extends TankDriveBase {
    private TalonSRX[] leftTalons, rightTalons, allTalons;
    private double conversionConstant;

  /**
   * 
   * @param leftMotors Array of left side drivebase motor controllers, must have length > 0
   * @param rightMotors Array of right side drivebase motor controllers, must have length > 0
   * @param inverted Invert the motors (make what would have been the front the back)
   * @param rightSideInverted Invert the right motor outputs to counteract them being flipped comparatively with the left ones
   * @param deadband A minimum motor input to move the drivebase
   * @param safetyEnabled Safety Mode, enforces motor safety which turns off the motors if communication lost, other failures, etc.
   */
    public SRXTankDrive() {
          super();
        
  }

  /**
   * Sets the Ramp Rate.
   * @param rampRate The time in seconds to go from zero to full speed.
   */
    public void setRampRate(double rampRate) {
        for(TalonSRX t : allTalons) {
            t.configOpenloopRamp(rampRate, 0);
        }
  }

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

  public void initBrakeMode(){
    for (TalonSRX t: allTalons) {
        t.setNeutralMode(NeutralMode.Brake);
      }
  }

    public void initCoastMode() {
        for (TalonSRX t: allTalons) {
            t.setNeutralMode(NeutralMode.Coast);
          }
  }

  /**
   * Sets the velocities of the left and right motors of the robot.
   * @param leftVel  Velocity for left motors in inches/sec
   * @param rightVel Velocity for right motors in inches/sec
   */
    public void setVelocityControl(double leftVel, double rightVel) {
  }

  /**
   * Sets the PIDControllers setpoints for the left and right side motors to the
   * given positions in inches forward.
   * 
   * @param deltaLeft  The desired change in left position in inches
   * @param deltaRight The desired change in right position in inches
   */
    public void setEncodersPositionControl(double deltaLeft, double deltaRight) {
      
  }

    public double inchesToTicks(double inches) {
        return inches / (Math.PI * conversionConstant);
  }

    public double ticksToInches(double revolutions) {
        return revolutions * Math.PI * conversionConstant;
  }

  public double getLeftPositionTicks(){
        return leftTalons[0].getSensorCollection().getQuadraturePosition();
  }

    public double getRightPositionTicks() {
        return rightTalons[0].getSensorCollection().getQuadraturePosition();
  }

  public double getLeftPositionInches() {
    return ticksToInches(getLeftPositionTicks());
  }

  public double getRightPositionInches() {
    return ticksToInches(getRightPositionTicks());
  }

  /**
   * @return Velocity of the left motors in ticks per second
   */
    public double getLeftVelocityTicks() {
        return leftTalons[0].getSensorCollection().getQuadratureVelocity();
  }

  /**
   * @return Velocity of the right motors in ticks per second
   */
    public double getRightVelocityTicks() {
        return rightTalons[0].getSensorCollection().getQuadratureVelocity();
  }

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

  @Override
  protected void initDefaultCommand() {
      setDefaultCommand(new XboxRocketLeagueDrive(this));
  }

}