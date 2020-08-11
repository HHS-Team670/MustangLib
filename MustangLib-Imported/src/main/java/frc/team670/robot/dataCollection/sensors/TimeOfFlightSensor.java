package frc.team670.robot.dataCollection.sensors;

import edu.wpi.first.wpilibj.I2C;

import java.util.TimerTask;

/**
 * I2C VL6180X Time of Flight sensor. Based on
 * //https://github.com/adafruit/Adafruit_VL6180X/blob/master/Adafruit_VL6180X.cpp.
 * 
 * @author riyagupta, meganchoy
 */
public class TimeOfFlightSensor {
    private I2C sensor;
    private java.util.Timer updater;
    private int range = ERROR;
    private boolean isHealthy;

    private static final int VL6180X_REG_RESULT_RANGE_STATUS = 0x04d;
    private static final int VL6180X_REG_SYSRANGE_START = 0x018;
    private static final int VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO = 0x04f;
    private static final int VL6180X_REG_RESULT_RANGE_VAL = 0x062;
    private static final int VL6180X_REG_SYSTEM_INTERRUPT_CLEAR = 0x015;
    private static final int ERROR = 300;

    private final static int TOF_ADDR = 0x29;

    /**
     * @param horizontalOffset horizontal offset from the center of the robot on the
     *                         side it is on. Left is negative, right is positive.
     */
    public TimeOfFlightSensor(I2C.Port port) {
        sensor = new I2C(port, TOF_ADDR);
        updater = new java.util.Timer();
        isHealthy = true;
        initSensor();
        start();
    }

    private void initSensor() {
        write(0x0207, 0x01);
        write(0x0208, 0x01);
        write(0x0096, 0x00);
        write(0x0097, 0xfd);
        write(0x00e3, 0x00);
        write(0x00e4, 0x04);
        write(0x00e5, 0x02);
        write(0x00e6, 0x01);
        write(0x00e7, 0x03);
        write(0x00f5, 0x02);
        write(0x00d9, 0x05);
        write(0x00db, 0xce);
        write(0x00dc, 0x03);
        write(0x00dd, 0xf8);
        write(0x009f, 0x00);
        write(0x00a3, 0x3c);
        write(0x00b7, 0x00);
        write(0x00bb, 0x3c);
        write(0x00b2, 0x09);
        write(0x00ca, 0x09);
        write(0x0198, 0x01);
        write(0x01b0, 0x17);
        write(0x01ad, 0x00);
        write(0x00ff, 0x05);
        write(0x0100, 0x05);
        write(0x0199, 0x05);
        write(0x01a6, 0x1b);
        write(0x01ac, 0x3e);
        write(0x01a7, 0x1f);
        write(0x0030, 0x00);
        // Recommended : Public registers - See data sheet for more detail
        write(0x0011, 0x10); // Enables polling for 'New Sample ready'
        // when measurement completes
        write(0x010a, 0x30); // Set the averaging sample period
        // (compromise between lower noise and
        // increased execution time)
        write(0x003f, 0x46); // Sets the light and dark gain (upper
        // nibble). Dark gain should not be
        // changed.
        write(0x0031, 0xFF); // sets the # of range measurements after
        // which auto calibration of system is
        // performed
        write(0x0040, 0x63); // Set ALS integration time to 100ms
        write(0x002e, 0x01); // perform a single temperature calibration
        // of the ranging sensor

        // Optional: Public registers - See data sheet for more detail
        write(0x001b, 0x09); // Set default ranging inter-measurement
        // period to 100ms
        write(0x003e, 0x31); // Set default ALS inter-measurement period
        // to 500ms
        write(0x0014, 0x24); // Configures interrupt on 'New Sample
        // Ready threshold event'
    }

    /**
     * @return the distance from the sensor, in mm
     */
    public int getDistance() {
        return range;
    }

    public boolean isHealthy() {
        return isHealthy;
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

    public void stop() {
        updater.cancel();
        updater = new java.util.Timer();
    }

    private boolean write(int registerAddress, int data) {
        byte[] rawData = new byte[3];

        rawData[0] = (byte) ((registerAddress >> 8) & 0xFF); // MSB of register Address
        rawData[1] = (byte) (registerAddress & 0xFF); // LSB of register address
        rawData[2] = (byte) data;

        if (!sensor.writeBulk(rawData, 3)) {
            isHealthy = true;
            return false;
        }

        isHealthy = false;
        return true;
    }

    private int readShortInt(int registerAddress) {
        byte[] data = new byte[1];

        // This sensor needs 2 bytes so cannot just use read method on I2C class
        byte[] rawData = new byte[2];

        rawData[0] = (byte) ((registerAddress >> 8) & 0xFF); // MSB of register Address
        rawData[1] = (byte) (registerAddress & 0xFF); // LSB of register address

        if (!sensor.transaction(rawData, 2, data, 1)) {
            isHealthy = true;
            return data[0] & 0xFF;
        }

        isHealthy = false;
        return ERROR;
    }

    /**
     * Gets the time of flight distance unadjusted for offset and angle to target
     */
    private void update() {
        // wait for device to be ready for range measurement
        while ((readShortInt(VL6180X_REG_RESULT_RANGE_STATUS) & 0x01) == 0)
            ;

        // Start a range measurement
        write(VL6180X_REG_SYSRANGE_START, 0x01);

        // Poll until bit 2 is set
        while ((readShortInt(VL6180X_REG_RESULT_INTERRUPT_STATUS_GPIO) & 0x04) == 0)
            ;

        // read range in mm
        range = readShortInt(VL6180X_REG_RESULT_RANGE_VAL);

        // clear interrupt
        write(VL6180X_REG_SYSTEM_INTERRUPT_CLEAR, 0x07);
    }

}