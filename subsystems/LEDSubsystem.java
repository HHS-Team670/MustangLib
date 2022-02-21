package frc.team670.mustanglib.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.team670.mustanglib.utils.LEDColor;

/**
 * Represents an addresable LED strip
 * 
 * @author AkshatAdsule, LakshBhambhani, VeganBurg
 */
public abstract class LEDSubsystem extends MustangSubsystemBase {
    /**
     * The default duration of a blink
     */
    public static final int DEFAULT_BLINK_DURATION = 20;

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;

    private int m_rainbowFirstPixelHue;

    private int blinkCounter;
    private int blinkEndCount;
    protected boolean isBlinking;
    private LEDColor blinkColor;

    /**
     * Creates a new LEDSubsystem
     * @param port The port at which the LED strip is connected to
     * @param length The number of LEDS in the LED strip
     */
    public LEDSubsystem(int port, int length) {
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
        if(isBlinking) { 
            blinkCounter++;
            if(blinkCounter >= blinkEndCount) {
                for(int i = 0; i < m_ledBuffer.getLength(); i++) {
                    m_ledBuffer.setHSV(i, blinkColor.h, blinkColor.s, blinkColor.v);
                }
            }
            if(blinkCounter >= blinkEndCount * 2) {
                blinkCounter = 0;
                isBlinking = false;
            }
        }
        m_led.setData(m_ledBuffer);
    }

    @Override
    public void debugSubsystem() {}

    @Override
    public HealthState checkHealth() {
        return HealthState.GREEN;
    }

    /**
     * Creates a rainbow effect on the LED Strip
     */
    public void rainbow() {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            // Set the value
            m_ledBuffer.setHSV(i, hue, 255, 255);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
    }

    /**
     * Changes the LED strip so that all the LEDs are one color
     * @param Color The color
     */
    public void solid(LEDColor color){
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setHSV(i, color.h, color.s, color.v);
        }
    }


    /**
     * Makes the LED strip blink for a certain duration
     * @param color The color to blink with
     * @param duration The duration of the blink
     */
    public void blink(LEDColor color, int duration) {
        if(!isBlinking) {
            blinkCounter = 0;
            isBlinking = true;
            blinkColor = color;
            blinkEndCount = duration;
            for(int i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setHSV(i, color.h, color.s, color.v * 2);
            }
        }
    }

    /**
     * Makes the LED strip blink for {@link #DEFAULT_BLINK_DURATION}.
     * @param color
     */
    public void blink(LEDColor color){
        blink(color, DEFAULT_BLINK_DURATION);
    }

    /**
     * "Fills" the led strip with the active color, proportional to the ratio given
     * @param active The "filled" color
     * @param inactive The "empty" color
     * @param ratio The ratio of the active color vs the inactive color
     */
    public void progressBar(LEDColor active, LEDColor inactive, double ratio) {
        ratio /= 2; // due to our leds having 2 leds per 'index'
        int ratioBright = (int)(m_ledBuffer.getLength() * ratio);

        for (int i = 0; i < ratioBright; i++) { // active
            m_ledBuffer.setHSV(i, active.h, active.s, active.v);
        }

        for (int i = ratioBright; i < m_ledBuffer.getLength(); i++) { // inactive
            m_ledBuffer.setHSV(i, inactive.h, inactive.s,  inactive.v);
        }
    }

    /**
     * "Fills" the led strip with the active color, proportional to the ratio given.
     * The inactive color will be dimmer than the given color
     * @param color The "filled" color
     * @param ratio The ratio between the active color vs the inactive color
     */
    public void progressBar(LEDColor color, double ratio){
        progressBar(color, LEDColor.dimmer(color), ratio);
    }

    /**
     * Sets the led buffer
     * @param buffer The new buffer
     */
    public void setBuffer(AddressableLEDBuffer buffer){
        this.m_ledBuffer = buffer;
    }

}
