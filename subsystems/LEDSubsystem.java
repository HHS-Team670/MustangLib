package frc.team670.mustanglib.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * Represents an addresable LED strip. Do not change to RGB or it will break
 * 
 * @author AkshatAdsule, edwar-d, LakshBhambhani
 */

 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

public abstract class LEDSubsystem extends MustangSubsystemBase {
    /**edu.wpi.first.wpilibj.examples.addressableled
     * The default duration of a blink
     */
    public static final int DEFAULT_BLINK_DURATION = 20;

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;

    private int startIndex;
    public int length;

    private int m_rainbowFirstPixelHue;
    private int m_mustangRainbowFirstSaturation;
    private LEDColor color = new LEDColor(0, 0, 0);
    private int blinkCounter;
    private LEDColor blinkColor;

    /**
     * Creates a new LEDSubsystem
     * 
     * @param port   The port at which the LED strip is connected to
     * @param length The number of LEDS in the LED strip
     */
    public LEDSubsystem(int port, int length) {
        this(port, 0, length);
    }

    public LEDSubsystem(int port, int startIndex, int endIndex) {
        length = endIndex - startIndex;
        m_led = new AddressableLED(port);
        m_ledBuffer = new AddressableLEDBuffer(length);
        m_led.setLength(length);

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();
    }
    /**
     * Instructions:
     * make a state enum for rainbow, blinking and solid/not moving
     * make setters for all enums (solid in solid hsv)
     * use similar the same logic that we used for the claw, only calling rainbow after a certain amount of cycles
     * ( we do this because we do not want to overload the robot by calling a method every .2 seconds when we could just do it every 3-5)
     * make a blinking method (could also just switch between solid hsv and off)
     * If you finish and it builds talk to me and we can test it out
     * Once your code works you can add whatever new methods and colors
     */

    public void rainbow() {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
          // Set the value
          m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
      }


    /**
     * Changes the LED strip so that all the LEDs are one solif color
     * Colors is in HSV FORMAT
     * 
     * @param Color The color
     **/

    public void solidhsv(LEDColor color) {
        if (this.color == null || !color.equals(this.color)) {
            this.color = color;
            for (var i = startIndex; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setHSV(i, (int) color.h, (int) color.s, (int) color.v);

            }
        }

    }

    public void mustangPeriodic() {
       // Fill the buffer with a rainbow
        rainbow();
        // Set the LEDs
        m_led.setData(m_ledBuffer);
    }

    @Override
    public void debugSubsystem() {
    }

    @Override
    public HealthState checkHealth() {
        return HealthState.GREEN;
    }

    /**
     * Sets the led buffer
     * 
     * @param buffer The new buffer
     */
    public void setBuffer(AddressableLEDBuffer buffer) {
        this.m_ledBuffer = buffer;
   }

    /**
     * Represents colors that our LED subsystem supports
     *Stores Colors in hsv format
     * 
     */
    public static class LEDColor {

        public static final LEDColor RED = new LEDColor(0);

        public static final LEDColor SPOOKY_ORANGE = new LEDColor(3);

        public static final LEDColor YELLOW = new LEDColor(15);

        public static final LEDColor GREEN = new LEDColor(60);

        public static final LEDColor LIGHT_BLUE = new LEDColor(85);

        public static final LEDColor BLUE = new LEDColor(120);

        public static final LEDColor PURPLE = new LEDColor(140);

        public static final LEDColor PINK = new LEDColor(169);

        public static final LEDColor WHITE = new LEDColor(0, 0);

        public static final LEDColor SEXY_PURPLE = new LEDColor(137);

        public static final LEDColor SEXY_YELLOW = new LEDColor(12);

        public final int h, s, v;

        public LEDColor(int h, int s, int v) {
            this.h = h;
            this.s = s;
            this.v = v;

        }

        public LEDColor(int h) {
            this(h, 255, 255);

        }

        public LEDColor(int h, int s) {
            this(h, s, 255);
        }

        public LEDColor dimmer() {
            return new LEDColor(h, s, v / 3);
        }

        public LEDColor brighter() {
            return new LEDColor(h, s, v * 3);
        }
    }

}
