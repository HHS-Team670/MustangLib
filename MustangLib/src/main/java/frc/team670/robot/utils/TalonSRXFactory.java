package frc.team670.robot.utils;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class TalonSRXFactory {

    public static TalonSRX buildTalonSRX(int deviceID){

        TalonSRX tsrx = new TalonSRX(deviceID);
        tsrx.configFactoryDefault();

        return tsrx;

    }

}