/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.mustanglib;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.team670.mustanglib.commands.MustangCommand;
import frc.team670.mustanglib.commands.MustangScheduler;
import frc.team670.mustanglib.dataCollection.PowerDistributionPanel;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the Robot periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public abstract class RobotContainerBase {

  // The robot's subsystems and commands are defined here...
  public static List<MustangSubsystemBase> allSubsystems = new ArrayList<MustangSubsystemBase>();
  private static PowerDistributionPanel pdp;

  private static boolean powerBudgetMonitoring = true;

  public RobotContainerBase(PowerDistributionPanel pdp){
    if(pdp.getType() == ModuleType.kCTRE){
      powerBudgetMonitoring = false;
    }
    this.pdp = pdp;
  }

  public static void addSubsystem(MustangSubsystemBase... subsystems) {
    for (MustangSubsystemBase m_subsystemBase : subsystems) {
      allSubsystems.add(m_subsystemBase);
    }
  }

  /**
   * Recalculates the health of all subsystems on the robot.
   */
  public static void checkSubsystemsHealth() {
    for (MustangSubsystemBase s : allSubsystems) {
      s.getHealth(true);
      if (s.getHealth(false).equals(HealthState.GREEN)) {
        // MustangScheduler.getInstance().registerSubsystem(s); //TODO: Test this today
        if(s.getDefaultMustangCommand() != null){
          MustangScheduler.getInstance().setDefaultCommand(s, s.getDefaultMustangCommand());
        }
      }
      else if (s.getHealth(false).equals(HealthState.RED)) {
        MustangScheduler.getInstance().cancel(s.getDefaultMustangCommand());
        MustangScheduler.getInstance().unregisterSubsystem(s);
      }
      s.pushHealthToDashboard();
    }
  }

  public abstract void robotInit();

  public abstract MustangCommand getAutonomousCommand();

  public abstract void autonomousInit();

  public abstract void teleopInit();

  public abstract void disabled();

  public abstract void periodic();

  public static List<MustangSubsystemBase> getSubsystems() {
    return allSubsystems;
  }

  public static void monitorBudget(){
    if(powerBudgetMonitoring){
      pdp.monitorBudget();
    }
  }

  public PowerDistributionPanel getPDP(){
    return pdp;
  }



}
