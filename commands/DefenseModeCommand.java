package frc.team670.mustanglib.commands;

import java.lang.reflect.Field;
import java.util.List;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;

public class DefenseModeCommand extends InstantCommand {
    List<MustangSubsystemBase> unusedSubsytems;
    boolean isActive;
    List<Integer> currentLimits;
    
    public DefenseModeCommand(List<MustangSubsystemBase> unusedSubsytems, List<Integer> currentLimits, boolean isActive){
        this.unusedSubsytems = unusedSubsytems;
        this.isActive = isActive;
        this.currentLimits = currentLimits;
    }

    @Override
    public void initialize(){
        for (int i = 0; i < unusedSubsytems.size(); i++){
            Field[] fields = unusedSubsytems.get(i).getClass().getDeclaredFields();
            for (Field field : fields) {
                Class<?> type = field.getType();
                if (type == SparkMAXLite.class){
                    field.setAccessible(true);
                    try {
                        SparkMAXLite sparkMax = (SparkMAXLite) field.get(unusedSubsytems.get(i));
                        sparkMax.setSmartCurrentLimit(currentLimits.get(i));
                    } catch (IllegalArgumentException e) {
                        e.printStackTrace();
                    } catch (IllegalAccessException e) {
                        e.printStackTrace();
                    }
                    

                    
                }
            }
        }
    }

}
