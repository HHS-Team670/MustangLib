package frc.team670.mustanglib.constants;

import edu.wpi.first.wpilibj.SerialPort;
import frc.team670.mustanglib.swervelib.Mk4iSwerveModuleHelper.GearRatio;

public class SwerveConfig {
    public double DRIVETRAIN_TRACKWIDTH_METERS; // Measure and set trackwidth

    public double DRIVETRAIN_WHEELBASE_METERS; // Measure and set wheelbase

    public int BACK_RIGHT_MODULE_DRIVE_MOTOR; // Set front left module drive motor ID
    public int BACK_RIGHT_MODULE_STEER_MOTOR; // Set front left module steer motor ID
    public int BACK_RIGHT_MODULE_STEER_ENCODER; // Set front left steer encoder ID
    public double BACK_RIGHT_MODULE_STEER_OFFSET; // Measure and set back right steer offset

    public int BACK_LEFT_MODULE_DRIVE_MOTOR; // Set front right drive motor ID
    public int BACK_LEFT_MODULE_STEER_MOTOR; // Set front right steer motor ID
    public int BACK_LEFT_MODULE_STEER_ENCODER; // Set front right steer encoder ID
    public double BACK_LEFT_MODULE_STEER_OFFSET; // Measure and set back left steer offset

    public int FRONT_RIGHT_MODULE_DRIVE_MOTOR; // Set back left drive motor ID
    public int FRONT_RIGHT_MODULE_STEER_MOTOR; // Set back left steer motor ID
    public int FRONT_RIGHT_MODULE_STEER_ENCODER; // Set back left steer encoder ID
    public double FRONT_RIGHT_MODULE_STEER_OFFSET; // Measure and set front right steer offset

    public int FRONT_LEFT_MODULE_DRIVE_MOTOR; // Set back right drive motor ID
    public int FRONT_LEFT_MODULE_STEER_MOTOR; // Set back right steer motor ID
    public int FRONT_LEFT_MODULE_STEER_ENCODER; // Set back right steer encoder ID
    public double FRONT_LEFT_MODULE_STEER_OFFSET; // Measure and set front left steer offset

    public double MAX_VELOCITY_METERS_PER_SECOND;
    public double MAX_VOLTAGE;

    public SerialPort.Port NAVX_PORT;
    public GearRatio SWERVE_MODULE_GEAR_RATIO;

    public SwerveConfig(double driveBaseTrackWidth, double driveBaseWheelBase, double maxVelocity, double maxVoltage,
            SerialPort.Port navXPort, GearRatio swerveModuleGearRatio,
            int frontLeftModuleDriveMotor, int frontLeftModuleSteerMotor, int frontLeftModuleSteerEncoder,
            double frontLeftModuleSteerOffset,
            int frontRightModuleDriveMotor, int frontRightModuleSteerMotor, int frontRightModuleSteerEncoder,
            double frontRightModuleSteerOffset,
            int backLeftModuleDriveMotor, int backLeftModuleSteerMotor, int backLeftModuleSteerEncoder,
            double backLeftModuleSteerOffset,
            int backRightModuleDriveMotor, int backRightModuleSteerMotor, int backRightModuleSteerEncoder,
            double backRightModuleSteerOffset) {
        DRIVETRAIN_TRACKWIDTH_METERS = driveBaseTrackWidth;
        DRIVETRAIN_WHEELBASE_METERS = driveBaseWheelBase;

        MAX_VELOCITY_METERS_PER_SECOND = maxVelocity;
        MAX_VOLTAGE = maxVoltage;

        NAVX_PORT = navXPort;

        FRONT_LEFT_MODULE_DRIVE_MOTOR = frontLeftModuleDriveMotor;
        FRONT_LEFT_MODULE_STEER_MOTOR = frontLeftModuleSteerMotor;
        FRONT_LEFT_MODULE_STEER_ENCODER = frontLeftModuleSteerEncoder;
        FRONT_LEFT_MODULE_STEER_OFFSET = frontLeftModuleSteerOffset;

        FRONT_RIGHT_MODULE_DRIVE_MOTOR = frontRightModuleDriveMotor;
        FRONT_RIGHT_MODULE_STEER_MOTOR = frontRightModuleSteerMotor;
        FRONT_RIGHT_MODULE_STEER_ENCODER = frontRightModuleSteerEncoder;
        FRONT_RIGHT_MODULE_STEER_OFFSET = frontRightModuleSteerOffset;

        BACK_LEFT_MODULE_DRIVE_MOTOR = backLeftModuleDriveMotor;
        BACK_LEFT_MODULE_STEER_MOTOR = backLeftModuleSteerMotor;
        BACK_LEFT_MODULE_STEER_ENCODER = backLeftModuleSteerEncoder;
        BACK_LEFT_MODULE_STEER_OFFSET = backLeftModuleSteerOffset;

        BACK_RIGHT_MODULE_DRIVE_MOTOR = backRightModuleDriveMotor;
        BACK_RIGHT_MODULE_STEER_MOTOR = backRightModuleSteerMotor;
        BACK_RIGHT_MODULE_STEER_ENCODER = backRightModuleSteerEncoder;
        BACK_RIGHT_MODULE_STEER_OFFSET = backRightModuleSteerOffset;
        SWERVE_MODULE_GEAR_RATIO = swerveModuleGearRatio;
    }

}
