package frc.team670.mustanglib.commands;

import java.lang.reflect.Field;
import java.util.List;
import java.util.Map;
import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;
import frc.team670.robot.constants.OI;

public class DefenseModeCommand extends Command implements MustangCommand  {
    List<CurrentLimiter> limiter;
    List<SparkMAXLite> list = new ArrayList<SparkMAXLite>();
    List<Integer> current = new ArrayList<Integer>();
    List<MustangSubsystemBase> testsubs = new ArrayList<MustangSubsystemBase>();
    Map<MustangSubsystemBase, HealthState> healthreq;
    boolean isActive;
    
    int index;


    public DefenseModeCommand(List<CurrentLimiter> limiter, boolean isActive){
        this.limiter = limiter;
        this.isActive = isActive;
        config();
        
    }

    public DefenseModeCommand(List<MustangSubsystemBase> unusedSubsytems, List<Integer> originalLimits , int constLimit,  boolean isActive){
        for (int i = 0; i < unusedSubsytems.size(); i++){
            limiter.set(i, new CurrentLimiter(unusedSubsytems.get(i), constLimit, originalLimits.get(i)));
        }
        this.isActive = isActive;
        config();
    }

    void config(){
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
        OI.defenseModeActive = !OI.defenseModeActive;
        index = 0;
    }

    public void runMain(int index){
        list.get(index).setSmartCurrentLimit(current.get(index));
    }

    @Override
    public void execute(){
        runMain(index);
        index++;
    }

    @Override
    public boolean isFinished(){
        return index == list.size() - 1;
    }

    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthreq;
    }

}
