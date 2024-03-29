/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.mustanglib.commands;

import java.util.Objects;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team670.mustanglib.utils.ConsoleLogger;

/**
 * A command that uses a RAMSETE controller ({@link RamseteController}) to follow a trajectory
 * {@link Trajectory} with a differential drive.
 *
 * <p>The command handles trajectory-following, PID calculations, and feedforwards internally.  This
 * is intended to be a more-or-less "complete solution" that can be used by teams without a great
 * deal of controls expertise.
 *
 * <p>Advanced teams seeking more flexibility (for example, those who wish to use the onboard
 * PID functionality of a "smart" motor controller) may use the secondary constructor that omits
 * the PID and feedforward functionality, returning only the raw wheel speeds from the RAMSETE
 * controller.
 */
@SuppressWarnings("PMD.TooManyFields")
public class RamseteCommand extends Command {
  private final Timer m_timer = new Timer();
  private final boolean m_usePID;
  private final Trajectory m_trajectory;
  private final Supplier<Pose2d> m_pose;
  private final RamseteController m_follower;
  private final SimpleMotorFeedforward m_rightFeedForward;
  private final SimpleMotorFeedforward m_leftFeedForward;
  private final DifferentialDriveKinematics m_kinematics;
  private final Supplier<DifferentialDriveWheelSpeeds> m_speeds;
  private final PIDController m_leftController;
  private final PIDController m_rightController;
  private final BiConsumer<Double, Double> m_output;
  private DifferentialDriveWheelSpeeds m_prevSpeeds;
  private double m_prevTime;

  /**
   * Constructs a new RamseteCommand that, when executed, will follow the provided trajectory.
   * PID control and feedforward are handled internally, and outputs are scaled -12 to 12
   * representing units of volts.
   *
   * <p>Note: The controller will *not* set the outputVolts to zero upon completion of the path -
   * this
   * is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory       The trajectory to follow.
   * @param pose             A function that supplies the robot pose - use one of
   *                         the odometry classes to provide this.
   * @param controller       The RAMSETE controller used to follow the trajectory.
   * @param leftFeedForward  The feedforward to use for left motors.
   * @param rightFeedForward The feedforward to use for right motors.
   * @param kinematics       The kinematics for the robot drivetrain.
   * @param wheelSpeeds      A function that supplies the speeds of the left and
   *                         right sides of the robot drive.
   * @param leftController   The PIDController for the left side of the robot drive.
   * @param rightController  The PIDController for the right side of the robot drive.
   * @param outputVolts      A function that consumes the computed left and right
   *                         outputs (in volts) for the robot drive.
   * @param requirements     The subsystems to require.
   */
  @SuppressWarnings("PMD.ExcessiveParameterList")
  
  public RamseteCommand(Trajectory trajectory,
                        Supplier<Pose2d> pose,
                        RamseteController controller,
                        SimpleMotorFeedforward leftFeedForward,
                        SimpleMotorFeedforward rightFeedForward,
                        DifferentialDriveKinematics kinematics,
                        Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds,
                        PIDController leftController,
                        PIDController rightController,
                        BiConsumer<Double, Double> outputVolts,
                        Subsystem... requirements) {
    m_trajectory = Objects.requireNonNull(trajectory);
    m_pose = Objects.requireNonNull(pose);
    m_follower = Objects.requireNonNull(controller);
    m_leftFeedForward = leftFeedForward;
    m_rightFeedForward = rightFeedForward;
    m_kinematics = Objects.requireNonNull(kinematics);
    m_speeds = Objects.requireNonNull(wheelSpeeds);
    m_leftController = Objects.requireNonNull(leftController);
    m_rightController = Objects.requireNonNull(rightController);
    m_output = Objects.requireNonNull(outputVolts);
    m_usePID = true;
    addRequirements(requirements);
  }

  

  @Override
  public void initialize() {
    m_prevTime = 0;
    var initialState = m_trajectory.sample(0);
    m_prevSpeeds = m_kinematics.toWheelSpeeds(
        new ChassisSpeeds(initialState.velocityMetersPerSecond,
            0,
            initialState.curvatureRadPerMeter
                * initialState.velocityMetersPerSecond));
    m_timer.reset();
    m_timer.start();
    if (m_usePID) {
      m_leftController.reset();
      m_rightController.reset();
    }
  }

  @Override
  public void execute() {

    double curTime = m_timer.get();
    double dt = curTime - m_prevTime;

    var targetWheelSpeeds = m_kinematics.toWheelSpeeds(
        m_follower.calculate(m_pose.get(), m_trajectory.sample(curTime)));

    var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
    var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

    double leftOutput;
    double rightOutput;

    if (m_usePID) {
      double leftFeedforward =
          m_leftFeedForward.calculate(leftSpeedSetpoint,
              (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);

      double rightFeedforward =
          m_rightFeedForward.calculate(rightSpeedSetpoint,
              (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);

      leftOutput = leftFeedforward
          + m_leftController.calculate(m_speeds.get().leftMetersPerSecond,
          leftSpeedSetpoint);

      rightOutput = rightFeedforward
          + m_rightController.calculate(m_speeds.get().rightMetersPerSecond,
          rightSpeedSetpoint);
    } else {
      leftOutput = leftSpeedSetpoint;
      rightOutput = rightSpeedSetpoint;
    }

    ConsoleLogger.consoleLog("LeftspeedSetPoint: %s, Left Speed: %s, LeftOutput: %s, rightSpeedSetPoint: %s Right Speed: %s rightOutput: %s", leftSpeedSetpoint, m_speeds.get().leftMetersPerSecond, leftOutput, rightSpeedSetpoint, m_speeds.get().rightMetersPerSecond, rightOutput);
    ConsoleLogger.consoleLog("Trajectory %s", m_trajectory.sample(curTime));
    ConsoleLogger.consoleLog("Pose: %s", m_pose.get().toString());

    m_output.accept(leftOutput, rightOutput);

    m_prevTime = curTime;
    m_prevSpeeds = targetWheelSpeeds;
  }

  @Override
  public void end(boolean interrupted) {
    ConsoleLogger.consoleLog("finished running trajectory");
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
  }
}
