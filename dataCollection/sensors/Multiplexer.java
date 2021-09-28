package frc.team670.mustanglib.dataCollection.sensors;

import java.util.HashMap;
import java.util.Map;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.I2C;
import frc.team670.mustanglib.utils.MustangNotifications;

/**
 * 
 * @author lakshbhambhani
 */
public class Multiplexer {
    
    private static I2C multiplexer;
    private static java.util.Timer updater;
    private final static int MULTI_ADDR = 0x70;
    Map<String, TimeOfFlightSensor> sensors = new HashMap<>();

    /**
     * @param port Port the sensor is connected to
     */
    public Multiplexer(I2C.Port port) {
        if(multiplexer != null){
            MustangNotifications.reportError("Please instantiate only 1 multiplexer");
            return;
        }
        multiplexer = new I2C(port, MULTI_ADDR); 

        updater = new java.util.Timer();
        start();
    }

    public void attachSensor(TimeOfFlightSensor newSensor, String key){
        sensors.put(key, newSensor);
        selectTOF(newSensor.getAddress());
        newSensor.initSensor();
    }

    public Map<String, TimeOfFlightSensor> getSensors(){
        return sensors;
    }

    private void start() {
        start(100);
    }

    private void start(int period) {
        TimerTask task = new TimerTask() {
            @Override
            public void run() {
                update();
            }
        };
        updater.scheduleAtFixedRate(task, 0, period);
    }

    private void update(){
        for (TimeOfFlightSensor sensor : sensors.values()) {
            selectTOF(sensor.getAddress());
            sensor.update();
        }
    }

    public void stop() {
        updater.cancel();
        updater = new java.util.Timer();
    }

    /**
    * Selects the given port on the multiplexer
    * @param address The address which should be selected. This should be in the range 0-7
    */
    private void selectTOF(int address) {
        // ensure address is between 0-7
        // Convert to binary to get what pins should be enabled
        byte[] rawData = new byte[1];
        rawData[0] = (byte)(1 << address);

        multiplexer.writeBulk(rawData, 1);

    }

}