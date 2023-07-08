package frc.team670.mustanglib.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * Represents an addresable LED strip. Do not change to RGB or it will break
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
        // ONLY for BLINK method
        if (isBlinking) {
            blinkCounter++;
            // sets the LED color to the inputted color when the `blinkCounter` is less than or equal to the `blinkEndCount`.
            // It sets the LED color to the specified color for the duration of the blink.
            if (blinkCounter <= blinkEndCount) {
                //sets led color to inputted color
                for (int i = startIndex; i < m_ledBuffer.getLength(); i++) {
                    m_ledBuffer.setHSV(i, (int) blinkColor.h, (int) blinkColor.s, (int) blinkColor.v);
                }
            }
            
            // This code block is responsible for turning off the blinking effect after the specified duration (`blinkEndCount`).
            if (blinkCounter > blinkEndCount) {
                for (int i = startIndex; i < m_ledBuffer.getLength(); i++) {
                    m_ledBuffer.setHSV(i, 0, 0, 0);
                }
                //blinkEndCounter = one blink (cycle of on and off)
                if (blinkCounter > blinkEndCount * 2) {
                    blinkCounter = 0;
                    isBlinking = false;
                }
            }

            m_led.setData(m_ledBuffer);
        }

        // responsible for updating the LED strip with any changes made to the LED buffer.
        // this DOES NOT apply to blink, but works for all other 
        // led methods that change the LED strip periodically
        
        if (changed) {
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
     * The function `solidRainbow` sets the color of each pixel in a LED strip to create a solid
     * rainbow effect.
     * 
     * @param isMaxBrightness A boolean value indicating whether the maximum brightness should be used
     * for the rainbow colors. If true, the brightness value will be set to 255. If false, the
     * brightness value will be set to 60.
     *
     * @param rainbowPixelHue The `rainbowPixelHue` parameter represents the starting hue value for the
     * rainbow effect. It determines the color of the first pixel in the rainbow sequence.
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
        changed = true;
    }

    /**
     * Creates an animated rainbow effect on the LED Strip
     */
   /**
    * The function `animatedRainbow` updates the rainbow animation by incrementing the hue of the first
    * pixel and calling the `solidRainbow` function.
    * 
    * @param isMaxBrightness A boolean value indicating whether the rainbow animation should be
    * displayed at maximum brightness or not.
    * @param updateSpeed The updateSpeed parameter determines how often the rainbow animation should be
    * updated. It represents the number of iterations before the animation is updated.
    * @param rainbowSpeed The `rainbowSpeed` parameter determines how quickly the colors of the rainbow
    * change. A higher value will result in a faster movement of the rainbow animation while a lower value
    * will result in a slower movement of the rainbow animation.
    */
    public void animatedRainbow(boolean isMaxBrightness, int updateSpeed, int rainbowSpeed) {
        if (animationCount > updateSpeed) {
            animationCount = 0;
            solidRainbow(false, m_rainbowFirstPixelHue + rainbowSpeed);
        }
        animationCount++;
    }

    /**
     * The function sets the HSV values of each pixel in a LED buffer to create a solid mustang
     * rainbow (white and green) effect with a variable saturation.
     * 
     * @param mustangRainbowFirstSaturation The parameter mustangRainbowFirstSaturation represents the
     * initial saturation value for the Mustang Rainbow effect. It determines the starting saturation
     * level for the colors in the rainbow effect.
     */

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

    /**
     * The function `animatedMustangRainbow` updates the saturation of a rainbow effect on a Mustang
     * car at a specified speed.
     * 
     * @param updateSpeed The updateSpeed parameter determines how often the animation should be
     * updated. It represents the number of iterations that need to pass before the animation is updated.
     * @param rainbowSpeed The rainbowSpeed parameter determines how quickly the saturation of the rainbow will change
     */
    public void animatedMustangRainbow(int updateSpeed, int rainbowSpeed) {
        if (animationCount > updateSpeed) {
            animationCount = 0;
            solidMustangRainbow(m_mustangRainbowFirstSaturation + rainbowSpeed);
        }
        animationCount++;
    }

    /**
     * The function checks if the color of an LED at a given index has changed.
     * 
     * @param index The index parameter represents the position of the LED in the LED buffer. It is
     * used to access the LED at the specified index in the buffer.
     * @param newColor The new color that you want to compare with the color of the LED at the
     * specified index.
     * @return The method is returning a boolean value.
     */
    private boolean colorChanged(int index, LEDColor newColor) {
        return !(m_ledBuffer.getLED(index).equals(newColor));
    }

    /**
     * Changes the LED strip so that all the LEDs are one solif color
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
