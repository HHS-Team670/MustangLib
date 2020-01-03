package frc.team670.robot.utils;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class VictorSPXFactory {

    public static VictorSPX buildVictorSPX(int deviceID) {

        VictorSPX vspx = new VictorSPX(deviceID);
        vspx.configFactoryDefault();

        return vspx;

    }


}