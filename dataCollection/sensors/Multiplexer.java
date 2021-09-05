package frc.team670.mustanglib.dataCollection.sensors;

import edu.wpi.first.wpilibj.DigitalOutput;

/**
 * Adafruit TCA9548A 1-to-8 I2C multiplexer. Based on
 * //https://learn.adafruit.com/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout/wiring-and-test
 * 
 * @author AkshatAdsule
 * @author KatiaBravo
 */
public class Multiplexer {
  /**
   * Represents inputs to a0, a1, and a2 pins on the multiplexer.
   * These pins allow switching of multiplexer channels
   */
  private DigitalOutput a0, a1, a2;

  public Multiplexer() {
    a0 = new DigitalOutput(3);
    a1 = new DigitalOutput(4);
    a2 = new DigitalOutput(5);
  }

  /**
   * Selects the given port on the multiplexer
   * @param address The address which should be selected. This should be in the range 0-7
   */
  public void select(int address) {
    // ensure address is between 0-7
    if(!(address <= 7 && address >= 0)) {
      return;
    }

    // Convert to binary to get what pins should be enabled
    String binary = Integer.toBinaryString(address);
    while(binary.length() < 3) {
      binary = "0" + binary;
    }

    a0.set(binary.charAt(2) == '1');
    a1.set(binary.charAt(1) == '1');
    a2.set(binary.charAt(0) == '1');
  }
}