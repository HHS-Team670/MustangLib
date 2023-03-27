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
    protected boolean isBlinking = false;
    private LEDColor blinkColor;
    private boolean changed = false;
    private boolean isMoving = false;
    private int animationCount = 0;

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

    public void mustangPeriodic() {
        // Handle turning off blink

        if (isBlinking) {
            blinkCounter++;
            if (blinkCounter <= blinkEndCount) {
                for (int i = startIndex; i < m_ledBuffer.getLength(); i++) {
                    m_ledBuffer.setHSV(i, (int) blinkColor.h, (int) blinkColor.s, (int) blinkColor.v);
                }
            }

            if (blinkCounter > blinkEndCount) {
                for (int i = startIndex; i < m_ledBuffer.getLength(); i++) {
                    m_ledBuffer.setHSV(i, 0, 0, 0);

                }
                if (blinkCounter > blinkEndCount * 2) {
                    blinkCounter = 0;
                    isBlinking = false;
                }
            }
            m_led.setData(m_ledBuffer);
        }

        if (isMoving || changed) {
            changed = false;
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
     * Creates a solid rainbow on the robot
     */
    public void solidRainbow(boolean isMaxBrightness, int rainbowPixelHue) {
        m_rainbowFirstPixelHue = rainbowPixelHue;
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
        isMoving = true;
        changed = true;
    }

    /**
     * Creates an animated rainbow effect on the LED Strip
     */
    public void animatedRainbow(boolean isMaxBrightness, int updateSpeed, int rainbowSpeed) {
        if (animationCount > updateSpeed) {
            animationCount = 0;
            solidRainbow(false, m_rainbowFirstPixelHue + rainbowSpeed);
        }
        animationCount++;
    }

    public void solidMustangRainbow(int mustangRainbowFirstSaturation) {
        m_mustangRainbowFirstSaturation = mustangRainbowFirstSaturation;
        // For every pixel
        color = null;
        for (var i = startIndex; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setHSV(i, 60, m_mustangRainbowFirstSaturation + (i * 255 / m_ledBuffer.getLength()) % 255, 255);
        }
        // Check bounds
        m_mustangRainbowFirstSaturation %= 255;
        changed = true;
    }

    public void animatedMustangRainbow(int updateSpeed, int rainbowSpeed) {
        if (animationCount > updateSpeed) {
            animationCount = 0;
            solidMustangRainbow(m_mustangRainbowFirstSaturation + rainbowSpeed);
        }
        animationCount++;
    }

    private boolean colorChanged(int index, LEDColor newColor) {
        return !(m_ledBuffer.getLED(index).equals(newColor));
    }

    /**
     * Changes the LED strip so that all the LEDs are one color
     * Colors is in HSV FORMAT
     * 
     * @param Color The color
     **/

    public void solidhsv(LEDColor color) {
        if (this.color == null || !color.equals(this.color)) {
            changed = true;
            this.color = color;
            for (var i = startIndex; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setHSV(i, (int) color.h, (int) color.s, (int) color.v);

            }
        }

    }

    /**
     * Changes the LED strip such that all LEDs are off
     */
    public void off() {
        solidhsv(new LEDColor(0, 0, 0));

    }

    /**
     * Makes the LED strip blink for a certain duration
     * 
     * @param color    The color to blink with
     * @param duration The duration of the blink
     */
    public void blinkhsv(LEDColor color, int duration) {
        if (!isBlinking) {
            this.color = null;
            blinkCounter = 0;
            isBlinking = true;
            blinkColor = color;
            blinkEndCount = duration;
            for (int i = startIndex; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setHSV(i, color.h, color.s, color.v);
            }
        }
        // changed = true;
    }

    /**
     * Makes the LED strip blink for {@link #DEFAULT_BLINK_DURATION}.
     * 
     * @param color
     */
    public void blinkhsv(LEDColor color) {
        blinkhsv(color, DEFAULT_BLINK_DURATION);
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
                m_ledBuffer.setHSV(i, (int) active.h, (int) active.s, (int) active.v);

            }
        }

        for (int i = ratioBright; i < length; i++) { // inactive
            if (colorChanged(i, inactive)) {
                m_ledBuffer.setHSV(i, (int) inactive.h, (int) inactive.s, (int) inactive.v);
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
