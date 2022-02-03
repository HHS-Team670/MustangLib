/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.mustanglib.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;

/**
 * Superclass for subsystems with rotation component
 */
public abstract class GravitySparkMaxRotatingSubsystem extends SubsystemBase implements TunableSubsystem {
    protected static final int NO_SETPOINT = 99999;
    protected SparkMAXLite rotator;
    protected RelativeEncoder encoder;
    protected int setpoint;
    protected double arbitraryFeedForwardConstant;
    protected int offsetFromEncoderZero;
    protected int PIDSlot;

    public GravitySparkMaxRotatingSubsystem(SparkMAXLite rotator, double arbitraryFeedForwardConstant, int PIDSlot, int forwardSoftLimit, int reverseSoftLimit, int forwardHardLimit, int reverseHardLimit, boolean reversed) {
        this.rotator = rotator;

        //sensor collection
        this.encoder = rotator.getEncoder();
        this.arbitraryFeedForwardConstant = arbitraryFeedForwardConstant;

        this.offsetFromEncoderZero = offsetFromEncoderZero;
        this.PIDSlot = PIDSlot;

        setpoint = RotatingSubsystem.NO_SETPOINT;

        if (reversed) {
            encoder.setPosition(reverseHardLimit);
        } else {
            encoder.setPosition(forwardHardLimit);
        }
            
        rotator.setSoftLimit(SoftLimitDirection.kForward, forwardSoftLimit);
        rotator.setSoftLimit(SoftLimitDirection.kReverse, reverseSoftLimit);
          
        rotator.setNeutralMode(NeutralMode.Brake);
    }  

    /**
     * Puts the main talon in percent output mode
     */
    public synchronized void stop() {
        clearSetpoint();
        rotator.set(ControlType.kDutyCycle, 0);
    }

    /**
     * Removes the setpoint for the talon on this subsystem
     */
    public void clearSetpoint() {
        setpoint = NO_SETPOINT;
    }

    /**
     * Updates the arbitrary feed forward on this subsystem
     */
    public synchronized void updateArbitraryFeedForward(){
        if(setpoint != NO_SETPOINT) {
            double value = getArbitraryFeedForwardAngleMultiplier() * arbitraryFeedForwardConstant;
            controller.setReference(setpoint, ControlType.kSmartMotion, PIDSlot, value);
          }
    }

    /**
     * Gets the multiplier for updating the arbitrary feed forward based on angle and subsystem
     */
    protected abstract double getArbitraryFeedForwardAngleMultiplier();

}