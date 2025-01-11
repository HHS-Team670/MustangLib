package frc.team670.mustanglib.swervelib.ctre;

import com.ctre.phoenix6.configs.Slot0Configs;

public class CTRModuleConfiguration {
    /** CAN ID of the drive motor */
    public int DriveMotorId = 0;
    /** CAN ID of the steer motor */
    public int SteerMotorId = 0;
    /** CAN ID of the CANcoder used for azimuth */
    public int CANcoderId = 0;
    /** Offset of the CANcoder in degrees */
    public double CANcoderOffset = 0;
    /** Gear ratio between drive motor and wheel */
    public double DriveMotorGearRatio = 0;
    /** Gear ratio between steer motor and CANcoder An example ratio for the SDS Mk4: 12.8 */
    public double SteerMotorGearRatio = 0;
    /** Wheel radius of the driving wheel in inches */
    public double WheelRadius = 0;
    /**
     * The location of this module's wheels relative to the physical center of the robot in meters
     * along the X axis of the robot
     */
    public double LocationX = 0;
    /**
     * The location of this module's wheels relative to the physical center of the robot in meters
     * along the Y axis of the robot
     */
    public double LocationY = 0;

    /** The steer motor gains */
    public Slot0Configs SteerMotorGains = new Slot0Configs();
    /** The drive motor gains */
    public Slot0Configs DriveMotorGains = new Slot0Configs();

    /** The maximum amount of current the drive motors can apply without slippage */
    public double SlipCurrent = 400;

    /** True if the steering motor is reversed from the CANcoder */
    public boolean SteerMotorReversed = false;

    public CTRModuleConfiguration withDriveMotorId(int id) {
        this.DriveMotorId = id;
        return this;
    }

    public CTRModuleConfiguration withSteerMotorId(int id) {
        this.SteerMotorId = id;
        return this;
    }

    public CTRModuleConfiguration withCANcoderId(int id) {
        this.CANcoderId = id;
        return this;
    }

    public CTRModuleConfiguration withCANcoderOffset(double offset) {
        this.CANcoderOffset = offset;
        return this;
    }

    public CTRModuleConfiguration withDriveMotorGearRatio(double ratio) {
        this.DriveMotorGearRatio = ratio;
        return this;
    }

    public CTRModuleConfiguration withSteerMotorGearRatio(double ratio) {
        this.SteerMotorGearRatio = ratio;
        return this;
    }

    public CTRModuleConfiguration withWheelRadius(double radius) {
        this.WheelRadius = radius;
        return this;
    }

    public CTRModuleConfiguration withLocationX(double locationXMeters) {
        this.LocationX = locationXMeters;
        return this;
    }

    public CTRModuleConfiguration withLocationY(double locationYMeters) {
        this.LocationY = locationYMeters;
        return this;
    }

    public CTRModuleConfiguration withSteerMotorGains(Slot0Configs gains) {
        this.SteerMotorGains = gains;
        return this;
    }

    public CTRModuleConfiguration withDriveMotorGains(Slot0Configs gains) {
        this.DriveMotorGains = gains;
        return this;
    }

    public CTRModuleConfiguration withSlipCurrent(double slipCurrent) {
        this.SlipCurrent = slipCurrent;
        return this;
    }

    public CTRModuleConfiguration withSteerMotorReversed(boolean steerMotorReversed) {
        this.SteerMotorReversed = steerMotorReversed;
        return this;
    }
}
