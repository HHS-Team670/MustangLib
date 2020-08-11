/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems.drivebase;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.SpeedController;
import frc.team670.robot.utils.PIDConstantSet;

/**
 * 
 * Represents a tank drive base using the SparkMax controllers. This can use either Brushed or Brushless motors
 * depending on what is specified in the constructor
 * Defaults to using XboxRocketLeagueDrive. This can be overriden using the setDefaultCommand() method.
 * 
 * @author shaylandias
 */
public abstract class SparkMaxTankDrive extends TankDriveBase {

    private CANSparkMax[] leftSparks, rightSparks, allSparks;

    private static final int VELOCITY_PID_SLOT = 1, POSITION_PID_SLOT = 2;

    private double conversionConstant;

    /**
     * 
     * @param leftMotors Array of left side drivebase motor controllers, must have length > 0
     * @param rightMotors Array of right side drivebase motor controllers, must have length > 0
     * @param inverted Invert the motors (make what would have been the front the back)
     * @param rightSideInverted Invert the right motor outputs to counteract them being flipped comparatively with the left ones
     * @param deadband A minimum motor input to move the drivebase
     * @param safetyEnabled Safety Mode, enforces motor safety which turns off the motors if communication lost, other failures, etc.
     * @param velocityPID PID values for velocity loop control
     * @param positionPID PID values for position loop control
     * @param conversionConstant Conversion constant for converting angular tick measurements to translational inches: (DriveBase Wheel Diameter) / (DriveBase Gear Ratio)
     * @param primaryCurrentLimit The current limit in Amps. The motor controller will reduce the controller voltage output to avoid surpassing this limit.
     * @param secondaryCurrentLimit The secondary current limit in Amps. The motor controller will disable the output of the controller *completely, but briefly* if the current limit is exceeded to reduce the current. 
     */
    public SparkMaxTankDrive(int[] leftMotorIds, int[] rightMotorIds, CANSparkMaxLowLevel.MotorType motorType, boolean inverted, 
                            boolean rightSideInverted, double deadband, boolean safetyEnabled, PIDConstantSet velocityPID, PIDConstantSet positionPID,
                            double conversionConstant, double rampRate, int primaryCurrentLimit, int secondaryCurrentLimit) {
        super();
        this.conversionConstant = conversionConstant;
        leftSparks = new CANSparkMax[leftMotorIds.length];
        rightSparks = new CANSparkMax[rightMotorIds.length];
        allSparks = new CANSparkMax[leftMotorIds.length + rightMotorIds.length];
        for(int i = 0; i < leftSparks.length; i++) {
            leftSparks[i] = new CANSparkMax(leftMotorIds[i], motorType);
            allSparks[i] = leftSparks[i];
            leftSparks[i].setInverted(inverted);
            leftSparks[i].setSmartCurrentLimit(primaryCurrentLimit);
            leftSparks[i].setSecondaryCurrentLimit(secondaryCurrentLimit);
            CANPIDController controller = leftSparks[i].getPIDController();
            controller.setP(velocityPID.P, VELOCITY_PID_SLOT);
            controller.setI(velocityPID.I, VELOCITY_PID_SLOT);
            controller.setD(velocityPID.D, VELOCITY_PID_SLOT);
            controller.setIZone(velocityPID.I_ZONE, VELOCITY_PID_SLOT);
            controller.setFF(velocityPID.FF, VELOCITY_PID_SLOT);
            
            controller.setP(positionPID.P, POSITION_PID_SLOT);
            controller.setI(positionPID.I, POSITION_PID_SLOT);
            controller.setD(positionPID.D, POSITION_PID_SLOT);
            controller.setIZone(positionPID.I_ZONE, POSITION_PID_SLOT);
            controller.setFF(positionPID.FF, POSITION_PID_SLOT);

            controller.setOutputRange(-1, 1);
            if(i > 0) {
                leftSparks[i].follow(leftSparks[0]);
            }
        }
        for(int i = 0; i < rightSparks.length; i++) {
            rightSparks[i] = new CANSparkMax(leftMotorIds[i], motorType);
            allSparks[i + leftMotorIds.length] = leftSparks[i];
            rightSparks[i].setInverted(inverted);
            rightSparks[i].setSmartCurrentLimit(primaryCurrentLimit);
            rightSparks[i].setSecondaryCurrentLimit(secondaryCurrentLimit);
            CANPIDController controller = rightSparks[i].getPIDController();
            controller.setP(velocityPID.P, VELOCITY_PID_SLOT);
            controller.setI(velocityPID.I, VELOCITY_PID_SLOT);
            controller.setD(velocityPID.D, VELOCITY_PID_SLOT);
            controller.setIZone(velocityPID.I_ZONE, VELOCITY_PID_SLOT);
            controller.setFF(velocityPID.FF, VELOCITY_PID_SLOT);
            
            controller.setP(positionPID.P, POSITION_PID_SLOT);
            controller.setI(positionPID.I, POSITION_PID_SLOT);
            controller.setD(positionPID.D, POSITION_PID_SLOT);
            controller.setIZone(positionPID.I_ZONE, POSITION_PID_SLOT);
            controller.setFF(positionPID.FF, POSITION_PID_SLOT);
            if(i > 0) {
                rightSparks[i].follow(rightSparks[0]);
            }
        }
        setRampRate(rampRate);
        setMotorControllers(leftSparks, rightSparks, false, rightSideInverted, deadband, safetyEnabled);
    }

    @Override
    public void setRampRate(double rampRate) {
        for(CANSparkMax spark : allSparks) {
            spark.setClosedLoopRampRate(rampRate);
            spark.setOpenLoopRampRate(rampRate);
        }
    }

    @Override
    public void initBrakeMode() {
        for(CANSparkMax spark : allSparks) {
            spark.setIdleMode(IdleMode.kBrake);
        }
    }

    @Override
    public void initCoastMode() {
        for(CANSparkMax spark : allSparks) {
            spark.setIdleMode(IdleMode.kCoast);
        }
    }

    @Override
    public void setVelocityControl(double leftVel, double rightVel) {
        leftSparks[0].getPIDController().setReference(leftVel, ControlType.kVelocity, VELOCITY_PID_SLOT);
        rightSparks[0].getPIDController().setReference(rightVel, ControlType.kVelocity, VELOCITY_PID_SLOT);
    }

    @Override
    public void setEncodersPositionControl(double deltaLeft, double deltaRight) {
        leftSparks[0].getPIDController().setReference(deltaLeft, ControlType.kPosition, POSITION_PID_SLOT);
        rightSparks[0].getPIDController().setReference(deltaRight, ControlType.kPosition, POSITION_PID_SLOT);
    }

    /**
     *  Note that ticks here is actually revolutions since that is what comes off of the SparkMax
     */
    public double inchesToTicks(double inches) {
        return inches / (Math.PI * conversionConstant);
    }

    /**
     *  Note that ticks here is actually revolutions since that is what comes off of the SparkMax
     */
    public double ticksToInches(double revolutions) {
        return revolutions * Math.PI * conversionConstant;
    }

    @Override
    public double getLeftPositionTicks() {
        return leftSparks[0].getEncoder().getPosition();
    }

    @Override
    public double getRightPositionTicks() {
        return rightSparks[0].getEncoder().getPosition();
    }

    @Override
    public double getLeftVelocityTicks() {
        return leftSparks[0].getEncoder().getVelocity();
    }

    @Override
    public double getRightVelocityTicks() {
        return rightSparks[0].getEncoder().getVelocity();
    }

    @Override
    public HealthState checkHealth() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void mustangPeriodic() {
        // TODO Auto-generated method stub

    }
    
}

