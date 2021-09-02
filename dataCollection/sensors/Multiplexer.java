package frc.team670.mustanglib.dataCollection.sensors;

import edu.wpi.first.wpilibj.I2C;
import frc.team670.mustanglib.utils.Logger;

/**
 * Adafruit TCA9548A 1-to-8 I2C multiplexer. Based on
 * //https://learn.adafruit.com/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout/wiring-and-test
 * 
 * @author AkshatAdsule
 * @author KatiaBravo
 */
public class Multiplexer {
  private I2C multiplexer;

  private static final int MUX_ADDR = 0x70;
  private static final int ERROR = 300;

  public Multiplexer(I2C.Port port) {
    multiplexer = new I2C(port, MUX_ADDR);
  }

  public void selectPort(int address) {
    assert address <= 7;
    multiplexer.write(MUX_ADDR, 1 << address);
  }

  public boolean write(int address, int data) {
      byte[] rawData = new byte[3];

        rawData[0] = (byte) ((address >> 8) & 0xFF); // MSB of register Address
        rawData[1] = (byte) (address & 0xFF); // LSB of register address
        rawData[2] = (byte) data;

        return !multiplexer.writeBulk(rawData, 3);
  }

  public int read(int address) {
    byte[] data = new byte[1];

    // This sensor needs 2 bytes so cannot just use read method on I2C class
    byte[] rawData = new byte[2];

    rawData[0] = (byte) ((address >> 8) & 0xFF); // MSB of register Address
    rawData[1] = (byte) (address & 0xFF); // LSB of register address

    if (multiplexer.transaction(rawData, 2, data, 1)) {
        return data[0] & 0xFF;
    }
    return ERROR;
  }

  public void testAddresses() {
    for (int t=0; t<8; t++) {
      selectPort(t);
      Logger.consoleLog("TCA Port #"); 
      Logger.consoleLog(Integer.toString(t));

      for (int addr = 0; addr<=127; addr++) {
        if (addr == MUX_ADDR) {
          continue;
        }

       
        if (!write(addr, 0x01)) {
          Logger.consoleLog("Found I2C 0x");  
          Logger.consoleLog(Integer.toBinaryString(addr));
        }
      }
    }
  }
}