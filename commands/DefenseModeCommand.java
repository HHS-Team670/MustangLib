package frc.team670.mustanglib.commands;

import java.lang.reflect.Field;
import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;

public class DefenseModeCommand extends Command {
    List<CurrentLimiter> limiter;
    boolean isActive;
    List<SparkMAXLite> list;
    List<Integer> current;
    int index;


    public DefenseModeCommand(List<CurrentLimiter> limiter, boolean isActive){
        this.isActive = isActive;
        this.limiter = limiter;
        for (int i = 0; i < limiter.size(); i++){
            Field[] fields = limiter.get(i).subsytem.getClass().getDeclaredFields();
            for (Field field : fields) {
                Class<?> type = field.getType();
                if (type == SparkMAXLite.class){
                    field.setAccessible(true);
                    try {
                        SparkMAXLite sparkMax = (SparkMAXLite) field.get(limiter.get(i).subsytem);
                        if (isActive) {
                            list.add(sparkMax);
                            current.add(limiter.get(i).limit);
                        } else {
                            list.add(sparkMax);
                            current.add(limiter.get(i).orginalLimit);
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

    public DefenseModeCommand(List<MustangSubsystemBase> unusedSubsytems, List<Integer> originalLimits , int constLimit,  boolean isActive){
        this.isActive = isActive;
        for (int i = 0; i < unusedSubsytems.size(); i++){
            limiter.set(i, new CurrentLimiter(unusedSubsytems.get(i), constLimit, originalLimits.get(i)));
        }
        for (int i = 0; i < limiter.size(); i++){
            Field[] fields = limiter.get(i).subsytem.getClass().getDeclaredFields();
            for (Field field : fields) {
                Class<?> type = field.getType();
                if (type == SparkMAXLite.class){
                    field.setAccessible(true);
                    try {
                        SparkMAXLite sparkMax = (SparkMAXLite) field.get(limiter.get(i).subsytem);
                        if (isActive) {
                            list.add(sparkMax);
                            current.add(limiter.get(i).limit);
                        } else {
                            list.add(sparkMax);
                            current.add(limiter.get(i).orginalLimit);
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


    @Override
    public void initialize(){
        index = 0;
    }

    @Override
    public void execute(){
        list.get(index).setSmartCurrentLimit(current.get(index));
        index++;
    }

    @Override
    public boolean isFinished(){
        return index == list.size() - 1;
    }

}
