/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.mustanglib;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Objects;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
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

  private static final double startTimeMillis = System.currentTimeMillis();

  private static String timeSinceStartupAsString = "00:00:00.000";

  private static RobotBase instance;

  public static boolean overrideAtCompetition = true;

  RobotContainerBase robotContainer;

  public RobotBase(RobotContainerBase robotContainer) {
    this.robotContainer = robotContainer;
    RobotBase.instance = this;
  }

  public RobotContainerBase getRobotContainer() {
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

    // Log file setup. Logs can be found at /home/lvuser/logs
    SimpleDateFormat dateFormatter = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss");
    Date currentDate = new Date();
    String dateString = dateFormatter.format(currentDate);
    File logDirectory = new File("/home/lvuser/logs/" + dateString);

    // Deletes old log files
    // for(File directory : (new File("/home/lvuser/logs")).listFiles()) {
    // try {
    // Date directoryDate = dateFormatter.parse(directory.getName());
    // long timeDifferenceMillis = currentDate.getTime() - directoryDate.getTime();
    // if(timeDifferenceMillis > 3 * 24 * 3600 * 1000) { //If too much time has
    // passed (in millisecond), delete directory
    // recursivelyDeleteDirectory(directory);
    // }
    // } catch (ParseException e) {
    // Logger.consoleError("Failed to parse date from log directory " +
    // directory.getName());
    // e.printStackTrace();
    // }
    // }

    // Creates log directory for the current code runthrough
    if (logDirectory.mkdirs()) {
      Logger.consoleLog("Successfully created logging directory at " + logDirectory.getAbsolutePath());
    } else {
      Logger.consoleLog("Failed to create logging directory at " + logDirectory.getAbsolutePath());
    }

    // Creates new subsystem files in the log directory
    for (MustangSubsystemBase subsystem : RobotContainerBase.allSubsystems) {
      subsystem.createLogFile(logDirectory);
    }
  }

  private static boolean recursivelyDeleteDirectory(File directoryToBeDeleted) {
    File[] allContents = directoryToBeDeleted.listFiles();
    if (allContents != null) {
      for (File file : allContents) {
        recursivelyDeleteDirectory(file);
      }
    }
    return directoryToBeDeleted.delete();
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
    updateTimeSinceStartupAsString();
    if (timer.hasElapsed(SYSTEM_CHECK_PERIOD)) {
      RobotContainerBase.checkSubsystemsHealth();
    }
  }

  private void updateTimeSinceStartupAsString() {
    int millisSinceStartup = (int) (System.currentTimeMillis() - startTimeMillis);
    int secondsSinceStartup = (int) (millisSinceStartup / 1000);
    int sec = secondsSinceStartup % 60;
    int min = (secondsSinceStartup / 60) % 60;
    int hours = (secondsSinceStartup / 60) / 60;
    int milliseconds = millisSinceStartup - ((millisSinceStartup / 1000) * 1000);
    String strSec = (sec < 10) ? "0" + Integer.toString(sec) : Integer.toString(sec);
    String strmin = (min < 10) ? "0" + Integer.toString(min) : Integer.toString(min);
    String strHours = (hours < 10) ? "0" + Integer.toString(hours) : Integer.toString(hours);
    String strMillis = (milliseconds < 100)
        ? "0" + ((milliseconds < 10) ? "0" + milliseconds : Integer.toString(milliseconds))
        : Integer.toString(milliseconds);
    timeSinceStartupAsString = strHours + ":" + strmin + ":" + strSec + "." + strMillis;
  }

  public static String getTimeSinceStartup() {
    return timeSinceStartupAsString;
  }

  public static synchronized RobotBase getInstance() {
    return Objects.requireNonNull(instance);
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
    robotContainer.disabledPeriodic();
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
    robotContainer.autonomousPeriodic();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      MustangScheduler.getInstance().cancel((MustangCommand) (m_autonomousCommand));
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
    robotContainer.teleopPeriodic();

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
