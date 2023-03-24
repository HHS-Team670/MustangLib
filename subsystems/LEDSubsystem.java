package frc.team670.mustanglib.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.team670.mustanglib.utils.LEDColor;
import edu.wpi.first.wpilibj.util.Color;

/**
 * Represents an addresable LED strip
 * 
 * @author AkshatAdsule, edwar-d, LakshBhambhani
 */
public abstract class LEDSubsystem extends MustangSubsystemBase {
    /**
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
    private int blinkEndCount;
    protected boolean isBlinking;
    private LEDColor blinkColor;
    private boolean changed = false;

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

    @Override
    public void mustangPeriodic() {
        // Handle turning off blink
        if (changed || isBlinking) {
            changed = false;
            if (isBlinking) {
                blinkCounter++;
                if (blinkCounter >= blinkEndCount) {
                    for (int i = startIndex; i < m_ledBuffer.getLength(); i++) {
                        m_ledBuffer.setRGB(i, (int) blinkColor.red, (int) blinkColor.green, (int) blinkColor.blue);
                    }
                }
                if (blinkCounter >= blinkEndCount * 2) {
                    blinkCounter = 0;
                    isBlinking = false;
                }
            }
            m_led.setData(m_ledBuffer);
        }
    }

    @Override
    public void debugSubsystem() {
    }

    @Override
    public HealthState checkHealth() {
        return HealthState.GREEN;
    }

    /**
     * Creates a rainbow effect on the LED Strip
     */
    public void rainbow(boolean isMaxBrightness) {
        // For every pixel
        color = null;
        int brightness = isMaxBrightness ? 255 : 60;
        for (var i = startIndex; i < m_ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            // Set the value

            m_ledBuffer.setHSV(i, hue, 255, brightness);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
        changed = true;
    }

    public void mustangRainbow() {
        // For every pixel
        color = null;
        for (var i = startIndex; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setHSV(i, 60, m_mustangRainbowFirstSaturation + (i * 255 / m_ledBuffer.getLength()) % 255, 255);
        }
        // Increase by to make the rainbow "move"
        m_mustangRainbowFirstSaturation += 5;
        // Check bounds
        m_mustangRainbowFirstSaturation %= 255;
        changed = true;
    }

    private boolean colorChanged(int index, LEDColor newColor) {
        return !(m_ledBuffer.getLED(index).equals(newColor));
    }

    /**
     * Changes the LED strip so that all the LEDs are one color
     * Colors is in RGB FORMAT
     * 
     * @param Color The color
     **/

    public void solidrgb(LEDColor color) {
        if (this.color == null || !color.equals(this.color)) {
            changed = true;
            this.color = color;
            for (var i = startIndex; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, (int) color.red, (int) color.green, (int) color.blue);

            }
        }

    }

    /**
     * Changes the LED strip such that all LEDs are off
     */
    public void off() {
        solidrgb(new LEDColor(0.0, 0.0, 0.0));

    }

    /**
     * Makes the LED strip blink for a certain duration
     * 
     * @param color    The color to blink with
     * @param duration The duration of the blink
     */
    public void blinkrgb(LEDColor color, int duration) {
        if (!isBlinking) {
            this.color = null;
            blinkCounter = 0;
            isBlinking = true;
            blinkColor = color;
            blinkEndCount = duration;
            for (int i = startIndex; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, (int) color.red, (int) color.green, (int) color.blue);
            }
        }
        changed = true;
    }

    /**
     * Makes the LED strip blink for {@link #DEFAULT_BLINK_DURATION}.
     * 
     * @param color
     */
    public void blinkrgb(LEDColor color) {
        blinkrgb(color, DEFAULT_BLINK_DURATION);
    }

    /**
     * "Fills" the led strip with the active color, proportional to the ratio given
     * 
     * @param active   The "filled" color
     * @param inactive The "empty" color
     * @param ratio    The ratio of the active color vs the inactive color
     */
    public void progressBar(LEDColor active, LEDColor inactive, double ratio) {
        ratio /= 2; // due to our leds having 2 leds per 'index'
        int ratioBright = (int) (length * ratio);
        color = null;
        for (int i = 0; i < ratioBright; i++) { // active
            if (colorChanged(i, active)) {
                changed = true;
                m_ledBuffer.setRGB(i, (int) active.red, (int) active.green, (int) active.blue);

            }
        }

        for (int i = ratioBright; i < length; i++) { // inactive
            if (colorChanged(i, inactive)) {
                m_ledBuffer.setRGB(i, (int) inactive.red, (int) inactive.green, (int) inactive.blue);
                changed = true;
            }
        }

    }

    /**
     * "Fills" the led strip with the active color, proportional to the ratio given.
     * The inactive color will be dimmer than the given color
     * 
     * @param color The "filled" color
     * @param ratio The ratio between the active color vs the inactive color
     */
    public void progressBar(LEDColor color, double ratio) {
        progressBar(color, color.dimmer(), ratio);
    }

    /**
     * Sets the led buffer
     * 
     * @param buffer The new buffer
     */
    public void setBuffer(AddressableLEDBuffer buffer) {
        this.m_ledBuffer = buffer;
        changed = true;
    }

}
