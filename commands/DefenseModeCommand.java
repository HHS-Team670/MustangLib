package frc.team670.mustanglib.commands;

import java.util.List;
import java.util.Map;


import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase.HealthState;
import frc.team670.mustanglib.utils.motorcontroller.SparkMAXLite;

public class DefenseModeCommand extends InstantCommand implements MustangCommand  {
    List<CurrentLimiter> limiter;
    List<SparkMAXLite> motorList = new ArrayList<SparkMAXLite>();
    List<Integer> speed = new ArrayList<Integer>();
    List<MustangSubsystemBase> testsubs = new ArrayList<MustangSubsystemBase>();
    Map<MustangSubsystemBase, HealthState> healthreq;
    


    public DefenseModeCommand(List<CurrentLimiter> limiter){
        this.limiter = limiter;
        config();
        
    }

    public DefenseModeCommand(List<MustangSubsystemBase> unusedSubsytems, int constLimit){
        
        //creates a CurrentLimiter using the paramiters pased to later be used in the config
        for (int i = 0; i < unusedSubsytems.size(); i++){
            limiter.set(i, new CurrentLimiter(unusedSubsytems.get(i), constLimit));
        }

        config();
    }

    void config(){
        for (int i = 0; i < limiter.size(); i++){
            // loop through subsytems and creates a list of their motors and a list of speeds for the motors
            // to be used when command is called
            SparkMAXLite[] motors = limiter.get(i).subsytem.getMotors();
            for (SparkMAXLite sparkMAXLite : motors) {
                motorList.add(sparkMAXLite);
                speed.add(limiter.get(i).limit);
            }
        }
    }

    
    @Override
    public void initialize(){
        for (int i = 0; i < motorList.size(); i++) {
            motorList.get(i).set(speed.get(i));
        }
    }


    @Override
    public Map<MustangSubsystemBase, HealthState> getHealthRequirements() {
        return healthreq;
    }

}