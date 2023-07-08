package frc.team670.mustanglib.subsystems.drivebase;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.subsystems.MustangSubsystemBaseIO;

public abstract class TankDriveIO extends MustangSubsystemBaseIO{

    protected DifferentialDrive drive;
    private MotorController leftMotors, rightMotors;
    @AutoLog
    public static class TankDriveIOInputs{
        

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
        this.leftMotors = leftMotor;
        this.rightMotors = rightMotor;
        drive = new DifferentialDrive(this.leftMotors, this.rightMotors);
        this.rightMotors.setInverted(true);
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
     * Sets the Ramp Rate.
     * 
     * @param rampRate The time in seconds to go from zero to full speed.
     */
    public abstract void setRampRate(double rampRate);

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



    /**
     * Sets the velocities of the left and right motors of the robot.
     * 
     * @param leftVel  Velocity for left motors in inches/sec
     * @param rightVel Velocity for right motors in inches/sec
     */
    public abstract void setVelocityControl(double leftVel, double rightVel);

    /**
     * Sets the PIDControllers setpoints for the left and right side motors to the
     * given positions in inches forward.
     * 
     * @param deltaLeft  The desired change in left position in inches
     * @param deltaRight The desired change in right position in inches
     */
    public abstract void setEncodersPositionControl(double deltaLeft, double deltaRight);

    public abstract double getLeftPositionTicks();

    public abstract double getRightPositionTicks();

    public double getLeftPositionInches() {
        return ticksToInches(getLeftPositionTicks());
    }

    public double getRightPositionInches() {
        return ticksToInches(getRightPositionTicks());
    }

    /**
     * @return Velocity of the left motors in ticks per second
     */
    public abstract double getLeftVelocityTicks();

    /**
     * @return Velocity of the right motors in ticks per second
     */
    public abstract double getRightVelocityTicks();

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

    public abstract double inchesToTicks(double inches);

    public abstract double ticksToInches(double ticks);
    /**
     * Checks if the Quick turn button binding is pressed. Quickturn enables a inplace tank drive rotation
     * @return true if quickturn is pressed, false otherwise
     */
   
    public abstract boolean isQuickTurnPressed();

    public abstract PIDController getLeftPIDController();

    public abstract PIDController getRightPIDController();

    public abstract SimpleMotorFeedforward getLeftSimpleMotorFeedforward();

    public abstract SimpleMotorFeedforward getRightSimpleMotorFeedforward();

    public abstract DifferentialDriveKinematics getKDriveKinematics();

    public abstract DifferentialDriveWheelSpeeds getWheelSpeeds();

    public abstract void tankDriveVoltage(double leftVoltage, double rightVoltage);

}

}
