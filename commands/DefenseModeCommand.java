package frc.team670.mustanglib.commands;

import java.lang.reflect.Field;
import java.util.List;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;

public class DefenseModeCommand extends InstantCommand {
    List<CurrentLimiter> limiter;
    boolean isActive;


    public DefenseModeCommand(List<CurrentLimiter> limiter, boolean isActive){
        this.isActive = isActive;
        this.limiter = limiter;
    }

    public DefenseModeCommand(List<MustangSubsystemBase> unusedSubsytems, List<Integer> originalLimits , int constLimit,  boolean isActive){
        this.isActive = isActive;
        for (int i = 0; i < unusedSubsytems.size(); i++){
            limiter.set(i, new CurrentLimiter(unusedSubsytems.get(i), constLimit, originalLimits.get(i)));
        }
    }


    @Override
    public void initialize(){

        for (int i = 0; i < limiter.size(); i++){
            Field[] fields = limiter.get(i).subsytem.getClass().getDeclaredFields();
            for (Field field : fields) {
                Class<?> type = field.getType();
                if (type == SparkMAXLite.class){
                    field.setAccessible(true);
                    try {
                        SparkMAXLite sparkMax = (SparkMAXLite) field.get(limiter.get(i).subsytem);
                        if (isActive) {
                            sparkMax.setSmartCurrentLimit(limiter.get(i).limit);
                        } else {
                            sparkMax.setSmartCurrentLimit(limiter.get(i).orginalLimit);
                        }
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
