/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems.drivebase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.SpeedController;

/**
 * 
 * Represents a tank drive base using the SparkMax controllers. This can use either Brushed or Brushless motors
 * depending on what is specified in the constructor
 * Defaults to using XboxRocketLeagueDrive. This can be overriden using the setDefaultCommand() method.
 * 
 * @author shaylandias
 */
public class SparkMaxTankDrive extends TankDriveBase {

    private CANSparkMax[] leftSparks, rightSparks;

    public SparkMaxTankDrive(int[] leftMotorIds, int[] rightMotorIds, CANSparkMaxLowLevel.MotorType motorType) {
        super(new SpeedController[leftMotorIds.length], new SpeedController[rightMotorIds.length]);
        leftSparks = new CANSparkMax[leftMotorIds.length];
        rightSparks = new CANSparkMax[rightMotorIds.length];
        for(int i = 0; i < leftSparks.length; i++) {
            leftSparks[i] = new CANSparkMax(leftMotorIds[i], motorType);
            // leftMotors. = leftSparks[i];

        }
    }

    @Override
    public void setRampRate(double rampRate) {

    }

    @Override
    public void initBrakeMode() {

    }

    @Override
    public void initCoastMode() {

    }

    @Override
    public void setVelocityControl(double leftVel, double rightVel) {

    }

    @Override
    public void setEncodersPositionControl(double deltaLeft, double deltaRight) {

    }

    @Override
    protected int inchesToTicks(double inches) {
        return 0;
    }

    @Override
    protected double ticksToInches(int ticks) {
        return 0;
    }

    @Override
    public int getLeftPositionTicks() {
        return 0;
    }

    @Override
    public int getRightPositionTicks() {
        return 0;
    }

    @Override
    public int getLeftVelocityTicks() {
        return 0;
    }

    @Override
    public int getRightVelocityTicks() {
        return 0;
    }
    
}

