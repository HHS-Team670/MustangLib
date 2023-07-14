/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.mustanglib.subsystems.drivebase;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.RobotConstantsBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.dataCollection.sensors.NavX;
import frc.team670.mustanglib.utils.functions.MathUtils;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.OI;
import frc.team670.robot.constants.RobotConstants;

/**
 * 
 * Represents a tank drive base using the WPIlib DifferentialDrive class.
 * 
 * @author shaylandias, lakshbhambhani, armaan, aditi
 */
public abstract class TankDrive extends DriveBase {
    private SparkMAXLite left1, left2, right1, right2;
    private RelativeEncoder left1Encoder, left2Encoder, right1Encoder, right2Encoder;
    public final DifferentialDriveKinematics kDriveKinematics;

  
  
    private List<SparkMAXLite> leftControllers, rightControllers;
    private List<SparkMAXLite> allMotors = new ArrayList<SparkMAXLite>();;
  
    private NavX navXMicro;
  
    private DifferentialDriveOdometry odometry;
  
    private MustangCommand defaultCommand;
  
    private SparkMaxPIDController leftPIDController;
    private SparkMaxPIDController rightPIDController;
    
    private double prevHeading;
    
    protected DifferentialDrive drive;
    // private MotorController leftMotors, rightMotors;
    private final Config kConfig;
    public static record Config(double kDriveBaseTrackWidth, double kDriveBaseGearRatio, double kDriveBaseWheelDiameter, 
    double kTrackWidthMeters, int kLeftLeaderMotorID, int kLeftFollowerMotorID, int kRightLeaderMotorID,
    int kRightFollowerMotorID, double deadband , boolean inverted, SerialPort.Port kNavPort, boolean kNavXReversed, AutonConfig autonConfig ) {
    }
    //Added an autonconfig to break up the config into easy to read chuncks. additionally, all of these values need to be updated whenever changes are amde to the tankdrive
    public static record AutonConfig(int kTimeoutMs, double leftKsVolts, double leftKvVoltSecondsPerMeter, 
    double leftKaVoltSecondsSquaredPerMeter,double rightKsVolts, double rightKvVoltSecondsPerMeter, 
    double rightKaVoltSecondsSquaredPerMeter, PIDController kLeftController, PIDController kRightController, double kMaxSpeedMetersPerSecond, double kMaxAccelerationMetersPerSecondSquared ){}
    public TankDrive(Config kConfig){
        this.kConfig = kConfig;
        kDriveKinematics = new DifferentialDriveKinematics(kConfig.kDriveBaseTrackWidth);
        /**
         * makes a spark max pair for the left motors
         */
        leftControllers = SparkMAXFactory.buildFactorySparkMAXPair(kConfig.kLeftLeaderMotorID, kConfig.kLeftFollowerMotorID,
        false, MotorConfig.Motor_Type.NEO);
        /**
         * makes a spark max pair for the right motors
         */       
        rightControllers = SparkMAXFactory.buildFactorySparkMAXPair(kConfig.kRightLeaderMotorID,
        kConfig.kRightFollowerMotorID, false, MotorConfig.Motor_Type.NEO);

        allMotors.addAll(leftControllers);
        allMotors.addAll(rightControllers);

        left1 = leftControllers.get(0);
        left2 = leftControllers.get(1);
        right1 = rightControllers.get(0);
        right2 = rightControllers.get(1);
        
        left1Encoder = left1.getEncoder();
        left2Encoder = left2.getEncoder();
        right1Encoder = right1.getEncoder();
        right2Encoder = right2.getEncoder();


        for(SparkMAXLite motorController : allMotors) {
            // Do NOT invert for the right side here
            motorController.getEncoder().setVelocityConversionFactor(getDrivebaseMetersPerRotation());
            motorController.getEncoder().setPositionConversionFactor(getDrivebaseVelocityConversionFactor());
          }
            // The DifferentialDrive inverts the right side automatically, however we want
        // invert straight from the Spark so that we can still use it properly with the
        // CANPIDController, so we need to tell differenetial drive to not invert.
        setMotorsInvert(leftControllers, false);
        setMotorsInvert(rightControllers, true); // Invert right controllers here so they will work properly with the CANPIDController
        setMotorControllers(left1, right1 ,false, false, .1, true);
        
        navXMicro = new NavX(kConfig.kNavPort);
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()),0,0, new Pose2d(0, 0, new Rotation2d()));
        prevHeading = getHeading();
        initBrakeMode();
    
        leftPIDController = left1.getPIDController();
        rightPIDController = right1.getPIDController();
        
        leftPIDController.setP(kConfig.autonConfig.kLeftController.getP());
        leftPIDController.setI(kConfig.autonConfig.kLeftController.getI());
        leftPIDController.setD(kConfig.autonConfig.kLeftController.getD());
    
        rightPIDController.setP(kConfig.autonConfig.kRightController.getP());
        rightPIDController.setI(kConfig.autonConfig.kRightController.getI());
        rightPIDController.setD(kConfig.autonConfig.kRightController.getD());
    
        leftPIDController.setOutputRange(-1, 1);
        rightPIDController.setOutputRange(-1, 1);
    }
    public double getDrivebaseMetersPerRotation(){
        return  ( (1 / kConfig.kDriveBaseGearRatio) * kConfig.kDriveBaseWheelDiameter * Math.PI * 0.0254);
    }
    public double getDrivebaseVelocityConversionFactor(){
        return getDrivebaseMetersPerRotation()/60;   
    }

    /**
     * This method is called by the constructor. Much of the time setup needs to be
     * performed on motors, so perform the setup in a subclass, then call this
     * method.
     * 
     * @param leftMotor        Leader of the left motors
     *                          
     * @param rightMotor       Leader of the right motors
     *
     * @param inverted          Invert the motors (make what would have been the
     *                          front the back)
     * @param rightSideInverted Invert the right motor outputs to counteract them
     *                          being flipped comparatively with the left ones
     * @param deadband          A minimum motor input to move the drivebase
     * @param safetyEnabled     Safety Mode, enforces motor safety which turns off
     *                          the motors if communication lost, other failures,
     *                          etc.
     */
    protected void setMotorControllers(MotorController leftMotor, MotorController rightMotor, boolean inverted,
            boolean rightSideInverted, double deadband, boolean safetyEnabled) {

        drive = new DifferentialDrive(leftMotor, rightMotor);
        rightMotor.setInverted(true);
        drive.setDeadband(deadband);
        drive.setSafetyEnabled(safetyEnabled);
    }

    /**
     * This method is called by the constructor or in a subclass if motor setup
     * needs to be performed. Much of the time setup needs to be performed on
     * motors, so perform the setup in a subclass, then call this method.
     * 
     * @param leftMotor  Leader of the left motors
     *
     * @param rightMotor Leader of the right motors
     */
    protected void setMotorControllers(MotorController leftMotor, MotorController rightMotor) {
        setMotorControllers(leftMotor, rightMotor, true, true, 0.02, true);
    }

    /**
     * This method is called by the constructor. Much of the time setup needs to be
     * performed on motors, so perform the setup in a subclass, then call this
     * method.
     * 
     * @param leftMotor        Leader of the left motors
     *
     * @param rightMotor       Leader of the rigth motors
     *
     * @param inverted          Invert the motors (make what would have been the
     *                          front the back)
     * @param rightSideInverted Invert the right motor outputs to counteract them
     *                          being flipped comparatively with the left ones
     */
    public void setMotorControllers(MotorController leftMotor, MotorController rightMotor, boolean inverted,
            boolean rightSideInverted) {
        setMotorControllers(leftMotor, rightMotor, inverted, rightSideInverted, 0.02, true);
    }

    /**
     * Checks the health for driveBase. RED if all motors are dead, GREEN if all
     * motors are alive, YELLOW if a motor is disconnected.
     *
     * @param isLeft1Error  boolean for left1 motor being errored
     * @param isLeft2Error  boolean for left2 motor being errored
     * @param isRight1Error boolean for right1 motor being errored
     * @param isRight2Error boolean for right2 motor being errored
     */
    public HealthState checkHealth(boolean isLeft1Error, boolean isLeft2Error, boolean isRight1Error,
            boolean isRight2Error) {
        HealthState state = HealthState.GREEN;

        if (!isLeft1Error && !isLeft2Error && !isRight1Error && !isRight2Error) {
            state = HealthState.GREEN;
        } else if (isLeft1Error && isLeft2Error && isRight1Error && isRight2Error) {
            state = HealthState.RED;
        } else {
            state = HealthState.YELLOW;
        }
        return state;
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
     * @param xSpeed        The forward throttle speed [-1, 1]
     * @param zRotation     The amount of rotation to turn [-1, 1] with positive
     *                      being
     *                      right
     * @param squaredInputs if squared, output much more smoother (quadratic
     *                      function)
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







    public double getLeftPositionInches() {
        return ticksToInches(getLeftPositionTicks());
    }

    public double getRightPositionInches() {
        return ticksToInches(getRightPositionTicks());
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

    public DifferentialDrive getDriveTrain() {
        return drive;
    }


 

  /**
   * Returns the wheel speeds of the leftMain Motor and rightMainMotor
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(left1Encoder.getVelocity(), right1Encoder.getVelocity());
  }



      /**
   * Makes the DriveBase's default command initialize teleop
   */
  public void initDefaultCommand(MustangCommand mCommand) {
    
    MustangScheduler.getInstance().setDefaultCommand(this, mCommand);
  }

  public void cancelDefaultCommand() {
    MustangScheduler.getInstance().cancel(defaultCommand);
  }

  /**
   * Checks the health for driveBase. RED if all motors are dead, GREEN if all
   * motors are alive and navx is connected, YELLOW if a motor is disconnected or
   * navX is not connected
   */
   
  public HealthState checkHealth() {
    HealthState motorHealth = checkHealth(left1.isErrored(), left2.isErrored(), right1.isErrored(), right2.isErrored());
    HealthState overallHealth;
    if(motorHealth == HealthState.GREEN && navXMicro != null)
      overallHealth = HealthState.GREEN;
    else if(motorHealth != HealthState.RED && navXMicro == null)
      overallHealth = HealthState.YELLOW;
    else
      overallHealth = motorHealth;

    return overallHealth;
  }

  /**
   * Sets all motors to Brake Mode
   */
  public void initBrakeMode() {
    setMotorsNeutralMode(allMotors, IdleMode.kBrake);
  }

  /**
   * Sets all motors to Coast Mode
   */
  public void initCoastMode() {
    setMotorsNeutralMode(allMotors, IdleMode.kCoast);
  }

  /**
   * Inverts a list of motors.
   */
  private void setMotorsInvert(List<SparkMAXLite> motorGroup, boolean invert) {
    for (CANSparkMax m : motorGroup) {
      m.setInverted(invert);
    }
  }

  /**
   * Sets all motors in the specified list to be in the specified mode
   * @param motors Motors to be set to a particular IdleMode
   * @param mode The target mode (coast or brake)
   */
  public void setMotorsNeutralMode(List<SparkMAXLite> motors, IdleMode mode) {
    for (CANSparkMax m : motors) {
      m.setIdleMode(mode);
    }
  }

  /**
   * Returns the Spark Max Encoder for the Left Main Motor
   */
  public RelativeEncoder getLeftMainEncoder() {
    return left1Encoder;
  }

  /**
   * Returns the Spark Max Encoder for the Left Follower Motor
   */
  public RelativeEncoder getLeftFollowerEncoder() {
    return left2Encoder;
  }

  /**
   * Returns the Spark Max Encoder for the Right Main Motor
   */
  public RelativeEncoder getRightMainEncoder() {
    return right1Encoder;
  }

  /**
   * Returns the Spark Max Encoder for the Right Follower Motor
   */
  public RelativeEncoder getRightFollowerEncoder() {
    return right2Encoder;
  }

  /**
   * Returns the Left Motor Controllers
   * 
   * @return The list of the motor controllers on the left side of the robot
   */
  public List<SparkMAXLite> getLeftControllers() {
    return leftControllers;
  }

  /**
   * Returns the Right Motor Controller
   * 
   * @return The list of the motor controllers on the right side of the robot
   */
  public List<SparkMAXLite> getRightControllers() {
    return rightControllers;
  }

  /**
   * Sets the ramp rate for the list of motors passed in.
   * @param rampRate The ramp rate in seconds from 0 to full throttle
   */
  public void setRampRate(List<SparkMAXLite> motors, double rampRate) {
    for (CANSparkMax m : motors) {
      m.setClosedLoopRampRate(rampRate);
      m.setOpenLoopRampRate(rampRate);
    }
  }

  /**
   * @param rampRate The ramp rate in seconds from 0 to full throttle
   */
  public void setTeleopRampRate() {
    setRampRate(allMotors, 0.36); // Will automatically cook some Cheezy Poofs
  }

  public void sendEncoderDataToDashboard() {
    SmartDashboard.putNumber("Left M Position Ticks", left1Encoder.getPosition());
    SmartDashboard.putNumber("Left M Velocity Ticks", left1Encoder.getVelocity());
    SmartDashboard.putNumber("Left S Position Ticks", left2Encoder.getPosition());
    SmartDashboard.putNumber("Left S Velocity Ticks", left2Encoder.getVelocity());
    SmartDashboard.putNumber("Right M Position Ticks", right1Encoder.getPosition());
    SmartDashboard.putNumber("Right M Velocity Ticks", right1Encoder.getVelocity());
    SmartDashboard.putNumber("Right S Position Ticks", right2Encoder.getPosition());
    SmartDashboard.putNumber("Right S Velocity Ticks", right2Encoder.getVelocity());
  }

   
  public void mustangPeriodic() {
    odometry.update(Rotation2d.fromDegrees(getHeading()), left1Encoder.getPosition(), right1Encoder.getPosition());
    if (SmartDashboard.getBoolean("aligned", false) && !MathUtils.doublesEqual(getHeading(), prevHeading, 0)) {
      SmartDashboard.putBoolean("aligned", false);
    }
    prevHeading = getHeading();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return current Pose2d, calculated by odometry
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose2d The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose2d) {
    // zeroHeading();
    navXMicro.reset(pose2d.getRotation().getDegrees() * (kConfig.kNavXReversed ? -1. : 1.));
    odometry.resetPosition(pose2d.getRotation(),0.0,0.0, pose2d);

    //If encoders aren't being properly zeroed, check if lE and rE are REVLibError.kOk
    REVLibError lE = left1Encoder.setPosition(0);
    REVLibError rE = right1Encoder.setPosition(0);
  }

  /**
   * Resets the odometry to 0 and zeroes the encoders.
   */
  public void resetOdometry() {
    zeroHeading();
    resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
  }

  /**
   * Returns the heading of the robot.
   * 
   * @return the robot's heading in degrees, in range [-180, 180]
   */
  public double getHeading() {
    return Math.IEEEremainder(navXMicro.getAngle(), 360) * (kConfig.kNavXReversed ? -1. : 1.);
  }



  /**
   * sets the postion of the left and right encoders to zero
   */
  public void zeroEncoders() {
    left1Encoder.setPosition(0);
    right1Encoder.setPosition(0);
  }

  /**
   * Sets the voltage for the left and right motors in a tank drive
   * system.
   * 
   * @param leftVoltage The voltage to be applied to the left side of the tank drive system. This will
   * control the speed and direction of the left side of the tank drive.
   * @param rightVoltage The rightVoltage parameter is the desired voltage to be applied to the right
   * side of the tank drive system.
   */
  public void tankDriveVoltage(double leftVoltage, double rightVoltage) {
    left1.setVoltage(leftVoltage);
    right1.setVoltage(rightVoltage);
  }

   
  /**
   * @return The left position of the drivebase in ticks.
   */
  
  public double getLeftPositionTicks() {
    return (int) (left1Encoder.getPosition() / RobotConstantsBase.TankDriveBase.kSparkTicksPerRotation);
  }

   
  /**
   * @return The method is returning the left velocity of the tank drive base in ticks per second.
   */
  public double getLeftVelocityTicks() {
    return (left1Encoder.getVelocity() / RobotConstantsBase.TankDriveBase.kSparkTicksPerRotation / 60);
  }

/**
   * @return The right position of the drivebase in ticks.
   */
  public double getRightPositionTicks() {
    return (int) (left1Encoder.getPosition() / RobotConstantsBase.TankDriveBase.kSparkTicksPerRotation);
  }

   
/**
   * @return The method is returning the right velocity of the tank drive base in ticks per second.
   */
  public double getRightVelocityTicks() {
    return (right1Encoder.getVelocity() / RobotConstantsBase.TankDriveBase.kSparkTicksPerRotation / 60);
  }

   
  public boolean isQuickTurnPressed() {
    return OI.getDriverController().getRightBumper();
  }

   
  public void setEncodersPositionControl(double leftPos, double rightPos) {
    left1Encoder.setPosition(leftPos);
    right1Encoder.setPosition(rightPos);
  }

   
  public void setRampRate(double rampRate) {
    left1.setOpenLoopRampRate(rampRate);
    right1.setOpenLoopRampRate(rampRate);
  }

   
  public void setVelocityControl(double leftSpeed, double rightSpeed) {
    left1.set(leftSpeed);
    right1.set(rightSpeed);
  }

  /**
   * Returns the velocity of the right side of the drivebase in inches/second from
   * the Spark Encoder
   */
   
  public double ticksToInches(double ticks) {
    double rotations = ticks / RobotConstantsBase.TankDriveBase.kSparkTicksPerRotation;
    return rotations * Math.PI * kConfig.kDriveBaseWheelDiameter;
  }

   
  public double inchesToTicks(double inches) {
    double rotations = inches / (Math.PI * kConfig.kDriveBaseWheelDiameter);
    return rotations * RobotConstantsBase.TankDriveBase.kSparkTicksPerRotation;
  }

   
  public void zeroHeading() {
    navXMicro.reset();
  }

   
  public DifferentialDriveKinematics getKDriveKinematics() {
    return kDriveKinematics;
  }

   
  public PIDController getLeftPIDController() {
    return kConfig.autonConfig.kLeftController;
  }

  public SparkMaxPIDController getLeftSparkMaxPIDController(){
    return leftPIDController;
  }

   
  public SimpleMotorFeedforward getLeftSimpleMotorFeedforward() {
    return new SimpleMotorFeedforward(kConfig.autonConfig.leftKsVolts, kConfig.autonConfig.leftKvVoltSecondsPerMeter,
    kConfig.autonConfig.leftKaVoltSecondsSquaredPerMeter);
  }

   
  public PIDController getRightPIDController() {
    return kConfig.autonConfig.kRightController;
  }

  public SparkMaxPIDController getRightSparkMaxPIDController(){
    return rightPIDController;
  }

   
  public SimpleMotorFeedforward getRightSimpleMotorFeedforward() {
    return new SimpleMotorFeedforward(kConfig.autonConfig.rightKsVolts, kConfig.autonConfig.rightKvVoltSecondsPerMeter,
    kConfig.autonConfig.rightKaVoltSecondsSquaredPerMeter);
  }

  /**
   * Toggles the idle of each motor from either kBrake or kCoast to the other one.
   */
   
  public void toggleIdleMode() {
    for (SparkMAXLite motor : allMotors) {
      if (motor.getIdleMode() == IdleMode.kBrake) {
        motor.setIdleMode(IdleMode.kCoast);
      } else {
        motor.setIdleMode(IdleMode.kBrake);
      }
    }
  }

  public void holdPosition() {
    getLeftSparkMaxPIDController().setReference(left1Encoder.getPosition(), CANSparkMax.ControlType.kPosition);
    getRightSparkMaxPIDController().setReference(right1Encoder.getPosition(), CANSparkMax.ControlType.kPosition);
  }

  public void releasePosition() {
    getLeftSparkMaxPIDController().setReference(0, CANSparkMax.ControlType.kDutyCycle);
    getRightSparkMaxPIDController().setReference(0, CANSparkMax.ControlType.kDutyCycle);
  }

   
  public void debugSubsystem() {
    SmartDashboard.putNumber("Heading", getHeading());
    SmartDashboard.putNumber("currentX", getPose().getX());
    SmartDashboard.putNumber("currentY", getPose().getY());
    SmartDashboard.putNumber("left 1 encoder", getLeftPositionTicks());
    SmartDashboard.putNumber("right 1 encoder", getRightPositionTicks());
    SmartDashboard.putNumber("left velocity", left1Encoder.getVelocity());
    SmartDashboard.putNumber("right velocity", right1Encoder.getVelocity());
    SmartDashboard.putNumber("left position", left1Encoder.getPosition());
    SmartDashboard.putNumber("right position", right1Encoder.getPosition());
    SmartDashboard.putNumber("Heading", getHeading());
    SmartDashboard.putNumber("pose X", getPose().getX());
    SmartDashboard.putNumber("pose Y", getPose().getY());
    sendEncoderDataToDashboard();
  }
}