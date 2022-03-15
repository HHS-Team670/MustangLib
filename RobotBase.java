/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.mustanglib;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.utils.Logger;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class RobotBase extends TimedRobot {

  private MustangCommand m_autonomousCommand;

  private Timer timer;
  private double SYSTEM_CHECK_PERIOD = 5;

  public static boolean overrideAtCompetition = true;

  RobotContainerBase robotContainer;

  public RobotBase(RobotContainerBase robotContainer){
    this.robotContainer = robotContainer;
  }

  public RobotContainerBase getRobotContainer(){
    return robotContainer;
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings
    RobotContainerBase.checkSubsystemsHealth();
    timer = new Timer();
    timer.start();
    robotContainer.robotInit();
    MustangScheduler.getInstance();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   * 
   * Re-calculates the health of all subsystems on the robot at specified
   * intervals.
   */
  @Override
  public void robotPeriodic() {
    MustangScheduler.getInstance().run();
    robotContainer.periodic();
    if (timer.hasElapsed(SYSTEM_CHECK_PERIOD)) {
      RobotContainerBase.checkSubsystemsHealth();
    }
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    robotContainer.disabled();
  }

  @Override
  public void disabledPeriodic() {

  }

  /**
   * This autonomous runs the autonomous command selected by your
   */
  @Override
  public void autonomousInit() {
    Logger.consoleLog("Autonomous Init");
    robotContainer.autonomousInit();
    m_autonomousCommand = robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      MustangScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    MustangScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      MustangScheduler.getInstance().cancel((MustangCommand)(m_autonomousCommand));
    }
    Logger.consoleLog("Teleop Init");
    robotContainer.teleopInit();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    MustangScheduler.getInstance().run();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    MustangScheduler.getInstance().cancelAll();
    Logger.consoleLog("Test Init");
    LiveWindow.setEnabled(false);
    robotContainer.testInit();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    MustangScheduler.getInstance().run();
  }

}
