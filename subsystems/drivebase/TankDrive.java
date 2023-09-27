
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.mustanglib.subsystems.drivebase;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.RobotConstantsBase;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.subsystems.drivebase.TankDriveIO.TankDriveIOInputs;
import frc.team670.mustanglib.utils.functions.MathUtils;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;

/**
 * 
 * Represents a tank drive base using the WPIlib DifferentialDrive class.
 * 
 * @author shaylandias, lakshbhambhani, armaan, aditi
 */
public abstract class TankDrive extends DriveBase {

    private TankDriveIO io;
    private TankDriveIOInputsAutoLogged inputs;
    public final DifferentialDriveKinematics kDriveKinematics;


    private DifferentialDriveOdometry odometry;
  
    private MustangCommand defaultCommand;
  
    private double prevHeading;

    // private MotorController leftMotors, rightMotors;
    private final Config kConfig;
    public static record Config(double kDriveBaseGearRatio, double kDriveBaseWheelDiameter, 
    double kTrackWidthMeters, int kLeftLeaderMotorID, int kLeftFollowerMotorID, int kRightLeaderMotorID,
    int kRightFollowerMotorID, double deadband , boolean inverted, SerialPort.Port kNavPort, boolean kNavXReversed, AutonConfig autonConfig) {
    }
    //Added an autonconfig to break up the config into easy to read chuncks. additionally, all of these values need to be updated whenever changes are amde to the tankdrive
    public static record AutonConfig(int kTimeoutMs, double leftKsVolts, double leftKvVoltSecondsPerMeter, 
    double leftKaVoltSecondsSquaredPerMeter,double rightKsVolts, double rightKvVoltSecondsPerMeter, 
    double rightKaVoltSecondsSquaredPerMeter, PIDController kLeftController, PIDController kRightController, double kMaxSpeedMetersPerSecond, double kMaxAccelerationMetersPerSecondSquared ){}
    public TankDrive(Config kConfig){
        super(new TankDriveIO(kConfig), new TankDriveIOInputsAutoLogged());
        this.kConfig = kConfig;
        this.io=(TankDriveIO)super.io;
        this.inputs=(TankDriveIOInputsAutoLogged)super.inputs;
        kDriveKinematics = new DifferentialDriveKinematics(kConfig.kTrackWidthMeters);
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(io.getHeading()),0,0, new Pose2d(0, 0, new Rotation2d()));
        prevHeading = io.getHeading();
        io.initBrakeMode();
    }
    public double getDrivebaseMetersPerRotation(){
        return  ( (1 / kConfig.kDriveBaseGearRatio) * kConfig.kDriveBaseWheelDiameter * Math.PI * 0.0254);
    }
    public double getDrivebaseVelocityConversionFactor(){
        return getDrivebaseMetersPerRotation()/60;   
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
        io.tankDrive(leftSpeed, rightSpeed, squaredInputs);
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
        io.curvatureDrive(xSpeed, zRotation, isQuickTurn);
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
        io.arcadeDrive(xSpeed, zRotation, squaredInputs);
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
        io.stop();
    }




   




 

  /**
   * Returns the wheel speeds of the leftMain Motor and rightMainMotor
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(inputs.leftVelocityRadPerSec/2.0/Math.PI*getDrivebaseMetersPerRotation(), inputs.rightVelocityRadPerSec/2.0/Math.PI*getDrivebaseMetersPerRotation());
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
   * Sets all motors to Brake Mode
   */
  public void initBrakeMode() {
    io.initBrakeMode();
  }

  /**
   * Sets all motors to Coast Mode
   */
  public void initCoastMode() {
    io.initCoastMode();
  }



  /**
   * Sets all motors in the specified list to be in the specified mode
   * 
   * @param mode The target mode (coast or brake)
   */
  public void setMotorsNeutralMode( IdleMode mode) {
    io.setMotorsNeutralMode(mode);
  }



  /**
   * Sets the ramp rate for the list of motors passed in.
   * @param rampRate The ramp rate in seconds from 0 to full throttle
   */
  public void setRampRate(List<SparkMAXLite> motors, double rampRate) {
    io.setRampRate(motors, rampRate);
  }

  /**
   * @param rampRate The ramp rate in seconds from 0 to full throttle
   */
  public void setTeleopRampRate() {
    io.setTeleopRampRate(); // Will automatically cook some Cheezy Poofs ?
  }

//   public void sendEncoderDataToDashboard() {
//     SmartDashboard.putNumber("Left M Position Ticks", left1Encoder.getPosition());
//     SmartDashboard.putNumber("Left M Velocity Ticks", left1Encoder.getVelocity());
//     SmartDashboard.putNumber("Left S Position Ticks", left2Encoder.getPosition());
//     SmartDashboard.putNumber("Left S Velocity Ticks", left2Encoder.getVelocity());
//     SmartDashboard.putNumber("Right M Position Ticks", right1Encoder.getPosition());
//     SmartDashboard.putNumber("Right M Velocity Ticks", right1Encoder.getVelocity());
//     SmartDashboard.putNumber("Right S Position Ticks", right2Encoder.getPosition());
//     SmartDashboard.putNumber("Right S Velocity Ticks", right2Encoder.getVelocity());
//   }

   
  public void mustangPeriodic() {
    odometry.update(Rotation2d.fromRadians(-inputs.gyroYawRad), inputs.leftPositionRad*kConfig.kDriveBaseWheelDiameter/2, inputs.rightPositionRad*kConfig.kDriveBaseWheelDiameter/2);
    if (SmartDashboard.getBoolean("aligned", false) && !MathUtils.doublesEqual(inputs.gyroYawRad, prevHeading, 0)) {
      SmartDashboard.putBoolean("aligned", false);
    }
    Logger.getInstance().recordOutput("Odometry", getPose());

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
    io.resetOdometry(pose2d);
    odometry.resetPosition(pose2d.getRotation(),0.0,0.0, pose2d);

   
  }

  /**
   * Resets the odometry to 0 and zeroes the encoders.
   */
  public void resetOdometry() {
    io.resetHeading(inputs);
    io.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
    
  }





  /**
   * sets the postion of the left and right encoders to zero
   */
  public void zeroEncoders() {
    io.zeroEncoders();
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
    io.tankDriveVoltage(leftVoltage, rightVoltage);
  }

   
  /**
   * @return The left position of the drivebase in rads.
   */
  
  public double getLeftPositionRad() {
    return inputs.leftPositionRad;
  }

   
  /**
   * @return The method is returning the left velocity of the tank drive base in rads per second.
   */
  public double getLeftVelocityRad() {
    return inputs.leftVelocityRadPerSec;
  }

/**
   * @return The right position of the drivebase in rad.
   */
  public double getRightPositionRad() {
    return inputs.rightPositionRad;
  }

   
/**
   * @return The method is returning the right velocity of the tank drive base in rads per second.
   */
  public double getRightVelocityRad() {
    return inputs.rightVelocityRadPerSec;
  }

   //The driver never uses quickturn
  public abstract boolean isQuickTurnPressed();

   
  

   
  public void setRampRate(double rampRate) {
    io.setRampRate(rampRate);
  }

   
  public void setVelocityControl(double leftSpeed, double rightSpeed) {
    io.setVelocityControl(leftSpeed, rightSpeed);
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

   
  public void resetHeading() {
    io.resetHeading(inputs);
  }

   
  public DifferentialDriveKinematics getKDriveKinematics() {
    return kDriveKinematics;
  }

   
  public PIDController getLeftPIDController() {
    return kConfig.autonConfig.kLeftController;
  }


  

   
  public SimpleMotorFeedforward getLeftSimpleMotorFeedforward() {
    return new SimpleMotorFeedforward(kConfig.autonConfig.leftKsVolts, kConfig.autonConfig.leftKvVoltSecondsPerMeter,
    kConfig.autonConfig.leftKaVoltSecondsSquaredPerMeter);
  }

   
  public PIDController getRightPIDController() {
    return kConfig.autonConfig.kRightController;
  }



   
  public SimpleMotorFeedforward getRightSimpleMotorFeedforward() {
    return new SimpleMotorFeedforward(kConfig.autonConfig.rightKsVolts, kConfig.autonConfig.rightKvVoltSecondsPerMeter,
    kConfig.autonConfig.rightKaVoltSecondsSquaredPerMeter);
  }

  /**
   * Toggles the idle of each motor from either kBrake or kCoast to the other one.
   */
   
  public void toggleIdleMode() {
    io.toggleIdleMode();
  }

 


   
  public void debugSubsystem() {
    // SmartDashboard.putNumber("Heading", getHeading());
    // SmartDashboard.putNumber("currentX", getPose().getX());
    // SmartDashboard.putNumber("currentY", getPose().getY());
    // SmartDashboard.putNumber("left 1 encoder", getLeftPositionTicks());
    // SmartDashboard.putNumber("right 1 encoder", getRightPositionTicks());
    // SmartDashboard.putNumber("left velocity", left1Encoder.getVelocity());
    // SmartDashboard.putNumber("right velocity", right1Encoder.getVelocity());
    // SmartDashboard.putNumber("left position", left1Encoder.getPosition());
    // SmartDashboard.putNumber("right position", right1Encoder.getPosition());
    // SmartDashboard.putNumber("Heading", getHeading());
    // SmartDashboard.putNumber("pose X", getPose().getX());
    // SmartDashboard.putNumber("pose Y", getPose().getY());
    // sendEncoderDataToDashboard();
  }

}