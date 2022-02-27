package frc.team670.mustanglib.dataCollection;

import java.util.List;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.mustanglib.utils.motorcontroller.TalonSRXLite;
import frc.team670.robot.RobotContainer;

public class PowerDistributionPanel extends PowerDistribution{

    private ModuleType type;
    private static int MAX_CURRENT_BUDGET;

    public PowerDistributionPanel(ModuleType type, int maxCurrentBudget){
        super(type == ModuleType.kCTRE ? 0 : 1, type);
        MAX_CURRENT_BUDGET = maxCurrentBudget;
        this.type = type;
    }

    public void monitorSubsystem(List<MotorController> subsystemControllers, double subsystemMaxAllowance){
        double currentUsage = 0;
        for(MotorController controller : subsystemControllers){
            if(controller instanceof SparkMAXLite){
                SparkMAXLite motor = (SparkMAXLite) controller; //not going to close the motor since its a reference to the original one
                currentUsage += motor.getOutputCurrent();
            }
            else if(controller instanceof TalonSRXLite){
                TalonSRXLite motor = (TalonSRXLite) controller; 
                currentUsage += motor.getStatorCurrent();
            }
        }
        if(getTotalCurrent() > MAX_CURRENT_BUDGET && currentUsage > subsystemMaxAllowance){
            
        }
    }

    public void reallocateBudget(MustangSubsystemBase overPoweredSubsystem){
         for(MustangSubsystemBase subsystem : RobotContainer.getSubsystems()){
            if(!subsystem.equals(overPoweredSubsystem)){
                for(MotorController controller : subsystem.getAllMotorControllers()){
                    if(controller instanceof SparkMAXLite){
                        SparkMAXLite motor = (SparkMAXLite) controller; //not going to close the motor since its a reference to the original one
                        motor.setSecondaryCurrentLimit(motor.get)
                    }
                    else if(controller instanceof TalonSRXLite){
                        TalonSRXLite motor = (TalonSRXLite) controller; 
                        motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration());
                    }
                }
            }
        }
    }

    public ModuleType getType(){
        return type;
    }

    public void monitorBudget(){
        for(MustangSubsystemBase subsystem : RobotContainer.getSubsystems()){
            monitorSubsystem(subsystem.getAllMotorControllers(), subsystem.getMaxCurrentAllowance());
        }
    }
    
}
