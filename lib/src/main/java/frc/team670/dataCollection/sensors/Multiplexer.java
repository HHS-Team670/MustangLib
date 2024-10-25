package frc.team670.mustanglib.dataCollection.sensors;

import java.util.HashMap;
import java.util.Map;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.I2C;
import frc.team670.mustanglib.utils.MustangNotifications;

/**
 * A class that routes multiple TOF sensors to a single I2C port to convserve bus space
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

    /**
     * The function attaches a TimeOfFlightSensor to a HashMap of sensors, selects the attached sensor,
     * and initializes it.
     * 
     * @param newSensor The new sensor object that you want to attach to the system.
     * @param key The "key" parameter is a string that serves as a unique identifier for the sensor
     * being attached. It is used as the key in a HashMap to store the sensor object.
     */
    public synchronized void attachSensor(TimeOfFlightSensor newSensor, String key){
        sensors.put(key, newSensor);
        selectTOF(newSensor.getAddress());
        newSensor.initSensor();
    }
    /**
     * 
     * @return the sensors this multiplexer is connected to
     */
    public Map<String, TimeOfFlightSensor> getSensors(){
        return sensors;
    }

    /**Starts the multiplexer's thread */
    private void start() {
        start(100);
    }

    /**Starts the multiplexer's thread with the specified periodic */
    private void start(int period) {
        TimerTask task = new TimerTask() {
            @Override
            public void run() {
                update();
            }
        };
        updater.scheduleAtFixedRate(task, 0, period);
    }

    /**
     * Updates all the sensors in the multiplexer
     */
    private synchronized void update(){
        for (TimeOfFlightSensor sensor : sensors.values()) {
            selectTOF(sensor.getAddress());
            sensor.update();
        }
    }
    /**
     * Stops the multiplexer from being updated
     */
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