package frc.team670.mustanglib.subsystems.drivebase;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.team670.mustanglib.dataCollection.sensors.NavX;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.drivebase.TankDrive.Config;
import frc.team670.mustanglib.utils.motorcontroller.MotorConfig;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXFactory;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;

public class TankDriveIO extends DriveBaseIO {

    private SparkMAXLite left1, left2, right1, right2;
    private RelativeEncoder left1Encoder, left2Encoder, right1Encoder, right2Encoder;
    private NavX navXMicro;
    private Config kConfig;
    private SparkMaxPIDController leftPIDController;
    private SparkMaxPIDController rightPIDController;
    protected DifferentialDrive drive;
    private List<SparkMAXLite> allMotors = new ArrayList<SparkMAXLite>();
   
    public TankDriveIO(Config kConfig){
        this.kConfig=kConfig;
         /**
         * makes a spark max pair for the left motors
         */
        List<SparkMAXLite> leftControllers = SparkMAXFactory.buildFactorySparkMAXPair(kConfig.kLeftLeaderMotorID(), kConfig.kLeftFollowerMotorID(),
        false, MotorConfig.Motor_Type.NEO);
        /**
         * makes a spark max pair for the right motors
         */       
        List<SparkMAXLite> rightControllers = SparkMAXFactory.buildFactorySparkMAXPair(kConfig.kRightLeaderMotorID(),
        kConfig.kRightFollowerMotorID(), false, MotorConfig.Motor_Type.NEO);
        allMotors.addAll(rightControllers);
        allMotors.addAll(leftControllers);
        left1 = leftControllers.get(0);
        left2 = leftControllers.get(1);
        right1 = rightControllers.get(0);
        right2 = rightControllers.get(1);
        left1Encoder = left1.getEncoder();
        left2Encoder = left2.getEncoder();
        right1Encoder = right1.getEncoder();
        right2Encoder = right2.getEncoder();
        left1Encoder.setPosition(0);
        left2Encoder.setPosition(0);
        right1Encoder.setPosition(0);
        right2Encoder.setPosition(0);
        setMotorsInvert(leftControllers, false);
        setMotorsInvert(rightControllers, true);
        setMotorControllers(left1, right1 ,false, false, .1, true);
        // for(SparkMAXLite motorController : allMotors) {
        //     // Do NOT invert for the right side here
        //     motorController.getEncoder().setVelocityConversionFactor(getDrivebaseMetersPerRotation(kConfig));
        //     motorController.getEncoder().setPositionConversionFactor(getDrivebaseVelocityConversionFactor(kConfig));
        //   }
        navXMicro = new NavX(kConfig.kNavPort());
        initBrakeMode();
    
        leftPIDController = left1.getPIDController();
        rightPIDController = right1.getPIDController();
        
        leftPIDController.setP(kConfig.autonConfig().kLeftController().getP());
        leftPIDController.setI(kConfig.autonConfig().kLeftController().getI());
        leftPIDController.setD(kConfig.autonConfig().kLeftController().getD());
    
        rightPIDController.setP(kConfig.autonConfig().kRightController().getP());
        rightPIDController.setI(kConfig.autonConfig().kRightController().getI());
        rightPIDController.setD(kConfig.autonConfig().kRightController().getD());
    
        leftPIDController.setOutputRange(-1, 1);
        rightPIDController.setOutputRange(-1, 1);
        navXMicro.reset();


    }

    public double getDrivebaseMetersPerRotation(Config kConfig){
        return  ( (1 / kConfig.kDriveBaseGearRatio()) * kConfig.kDriveBaseWheelDiameter() * Math.PI * 0.0254);
    }
    public double getDrivebaseVelocityConversionFactor(Config kConfig){
        return getDrivebaseMetersPerRotation(kConfig)/60;   
    }


    public double getHeading(){
        return navXMicro.getFusedHeading();
    }
    /**
   * @param rampRate The ramp rate in seconds from 0 to full throttle
   */
  public void setTeleopRampRate() {
    setRampRate(allMotors, 0.36); // Will automatically cook some Cheezy Poofs
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
    public void setMotorsInvert(List<SparkMAXLite> motorGroup, boolean invert) {
        for (CANSparkMax m : motorGroup) {
          m.setInverted(invert);
        }
      }

      /**
   * Sets all motors to Brake Mode
   */
  public void initBrakeMode() {
    setMotorsNeutralMode( IdleMode.kBrake);
  }
  /**
   * Sets all motors to Coast Mode
   */
  public void initCoastMode() {
    setMotorsNeutralMode( IdleMode.kCoast);
  }
  public void setVelocityControl(double leftSpeed, double rightSpeed) {
    left1.set(leftSpeed);
    right1.set(rightSpeed);
  }
  


  /**
   * Sets all motors in the specified list to be in the specified mode
   * @param motors Motors to be set to a particular IdleMode
   * @param mode The target mode (coast or brake)
   */
  public void setMotorsNeutralMode( IdleMode mode) {
   
    left1.setIdleMode(mode);
    left2.setIdleMode(mode);
    right1.setIdleMode(mode);
    right2.setIdleMode(mode);
    
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
    public void stop(){
        drive.stopMotor();
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


   
  public void setRampRate(double rampRate) {
    left1.setOpenLoopRampRate(rampRate);
    right1.setOpenLoopRampRate(rampRate);
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
   * Resets the odometry to the specified pose.
   *
   * @param pose2d The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose2d) {
    // zeroHeading();
    
    navXMicro.reset(pose2d.getRotation().getDegrees() * (kConfig.kNavXReversed() ? -1. : 1.));

    //If encoders aren't being properly zeroed, check if lE and rE are REVLibError.kOk
    REVLibError lE = left1Encoder.setPosition(0);
    REVLibError rE = right1Encoder.setPosition(0);
  }
    /**
   * sets the postion of the left and right encoders to zero
   */
  public void zeroEncoders() {
    left1Encoder.setPosition(0);
    right1Encoder.setPosition(0);
  }
 





    
    public void updateInputs(LoggableInputs input) {
        TankDriveIOInputs inputs=(TankDriveIOInputs)input;
        inputs.leftPositionRad = Units.rotationsToRadians(left1Encoder.getPosition() / kConfig.kDriveBaseGearRatio());
        inputs.rightPositionRad = Units.rotationsToRadians(right1Encoder.getPosition() / kConfig.kDriveBaseGearRatio());
        inputs.leftVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
            left1Encoder.getVelocity() / kConfig.kDriveBaseGearRatio());
        inputs.rightVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
            right1Encoder.getVelocity() / kConfig.kDriveBaseGearRatio());
        inputs.gyroYawRad = navXMicro.getFusedHeading()*Math.PI/180;
        inputs.gyroPitchRad=navXMicro.getPitch();
        
    }
    @AutoLog
    public static class TankDriveIOInputs extends DriveBaseIOInputs{
        public double leftPositionRad = 0.0;
        public double leftVelocityRadPerSec = 0.0;
        public double rightPositionRad = 0.0;
        public double rightVelocityRadPerSec = 0.0;
        public double gyroYawRad;
        public double gyroPitchRad;
        public void  updateFromSuper(){
          gyroYawRad = super.gyroYawRad;
          gyroPitchRad= super.gyroPitchRad;
      }

    }
    
    public void resetHeading() {
        navXMicro.reset();
    }
    @Override
    protected HealthState checkHealth() {
       
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
    @Override
    public void debugOutputs() {
       
    }
    
}
