package frc.team670.robot.utils;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SparkMAXFactory {

    public static CANSparkMax buildCANSparkMax(int deviceID, MotorType type) {

        CANSparkMax spmax = new CANSparkMax(deviceID, type);
        spmax.restoreFactoryDefaults();

        return spmax;

    }


}