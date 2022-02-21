package frc.team670.mustanglib.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.team670.mustanglib.utils.Logger;

public class LEDSubsystem extends MustangSubsystemBase {
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private int length;

    private int m_rainbowFirstPixelHue;
    private static final int DEFAULT_BRIGHTNESS = 128;
    private int blinkBrightness = DEFAULT_BRIGHTNESS;

    private boolean hasBlinked = false;

    private int debugCount = 0;

    private int blinkCounter;
    private int stopBlinkCount;
    private boolean isBlinking;
    private Color blinkColor;
    private static final int BLINK_DURATION = 20;

    private static AddressableLEDBuffer blankBuffer = new AddressableLEDBuffer(50);

    /**
     * Represents colors that our LED subsystem supports
     */
    public enum Color {
        BLUE(120), GREEN(56), RED(2), PURPLE(241);

        private final int h;

        Color(int h) {
            this.h = h;
        }

        /**
         * Gets the ID of the state.
         */
        public int getH() {
            return h;
        }

    }


    public LEDSubsystem(int port, int length) {
        m_led = new AddressableLED(9);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(70);
        m_led.setLength(m_ledBuffer.getLength());

        this.length = length;

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();

        for(int i = 0; i < blankBuffer.getLength(); i++) {
            blankBuffer.setRGB(i, 0, 0, 0);
        }
    }

    @Override
    public void mustangPeriodic() {
        if(isBlinking) {
            blinkCounter++;
            if(blinkCounter >= stopBlinkCount) {
                Logger.consoleError("Stopping blink. stop blink count: %n");
                isBlinking =  false;
                for(int i = 0; i < m_ledBuffer.getLength(); i++) {
                    m_ledBuffer.setHSV(i, blinkColor.getH(), 255, DEFAULT_BRIGHTNESS);
                }
            }
        }
        m_led.setData(m_ledBuffer);
    }

    @Override
    public void debugSubsystem() {
        if(debugCount >= 50) {
            debugCount = 0;
        }
        if(debugCount != 0) {
            blankBuffer.setRGB(debugCount - 1, 255, 0, 0);
        }
        blankBuffer.setRGB(debugCount, 0, 255, 255);
        debugCount++;
        m_ledBuffer = blankBuffer;
    }

    @Override
    public HealthState checkHealth() {
        return HealthState.GREEN;
    }

    public void rainbow() {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            // Set the value
            m_ledBuffer.setHSV(i, hue, 255, DEFAULT_BRIGHTNESS);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
    }

    public void solid(Color c){
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Set the value
            m_ledBuffer.setHSV(i, c.getH(), 255, DEFAULT_BRIGHTNESS);
        }
    }

    public void solid(Color c, int brightness){
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Set the value
            m_ledBuffer.setHSV(i, c.getH(), 255, brightness);
        }
    }

    public void blink(Color c){
        // // For every pixel
        // if(hasBlinked == false && blinkCounter < 100){
        //     for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        //         // Set the value
        //         m_ledBuffer.setHSV(i, c.getH(), 255, blinkBrightness);
        //     }
        //     blinkCounter++;
        // }
        // if(blinkCounter > 100){
        //     hasBlinked = true;
        // }
       
        // if(blinkBrightness < DEFAULT_BRIGHTNESS && hasBlinked == true){
        //     blinkBrightness *= 2;
        //     hasBlinked = false;
        // }
        // else{
        //     blinkBrightness /= 2;
        //     hasBlinked = true;
        // }

        // if(blinkCounter % BLINK_DURATION == 0) {
        //     if(blinkBrightness == DEFAULT_BRIGHTNESS) {
        //         blinkBrightness /= 2;
        //     } else {
        //         blinkBrightness *= 2;
        //     }
        //     for(int i = 0; i < m_ledBuffer.getLength(); i++) {
        //         m_ledBuffer.setHSV(i, c.getH(), 255, blinkBrightness);
        //     }
        // }
        // blinkCounter++;
        stopBlinkCount = BLINK_DURATION;
        isBlinking = true;
        blinkColor = Color.GREEN;
        blinkCounter = 0;
        Logger.consoleError("Starting blink. stop blink count: %d%n", stopBlinkCount);
        for(int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setHSV(i, c.getH(), 255, DEFAULT_BRIGHTNESS / 2);
        }
        m_led.setData(m_ledBuffer);
    }

    public void resourceBar(Color c, double ratioFilled){
        // For every pixel
        ratioFilled /= 2;
        int ratioBright = (int)(m_ledBuffer.getLength() * ratioFilled);
        Logger.consoleError("Setting ratio: 0 - %d: on, %d - %d off%n", ratioBright, ratioBright, m_ledBuffer.getLength());
        for (int i = 0; i < ratioBright; i++) {
            // Set the value
            // m_ledBuffer.setHSV(i, c.getH(), 255, DEFAULT_BRIGHTNESS);
            m_ledBuffer.setRGB(i, 255, 0, 0);
        }
        for (int j = ratioBright; j < m_ledBuffer.getLength(); j++) {
            // Logger.consoleError("Setting led at %d to dead",j);
            // Set the value
            // m_ledBuffer.setHSV(i, 0, 0, 30);
            m_ledBuffer.setRGB(j, 0, 255, 0);
        }
        m_led.setData(m_ledBuffer);
    }

    public void setBuffer(AddressableLEDBuffer buffer){
        this.m_ledBuffer = buffer;
    }

}
