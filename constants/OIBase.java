package frc.team670.mustanglib.constants;

import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.MustangController;

public abstract class OIBase {

  public OIBase(){
    
  }

    /**
   * Notifies the driver controller by rumbling it
   * 
   */
  public void notifyDriverController(MustangController controller, double power, double time) {
    rumbleController(controller, power, time);
    rumbleController(controller, 0, 1);
    rumbleController(controller, power, time);
  }

  public void rumbleController(MustangController controller, double power, double time) {
    controller.rumble(power, time);
  }

  public abstract void configureButtonBindings(MustangSubsystemBase... subsystemBases);


}