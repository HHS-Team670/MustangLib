package frc.team670.mustanglib.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Represents an LED Subsystem on the robot
 */
public class LEDSubsystem extends SubsystemBase {

    private AddressableLED led;
    private AddressableLEDBuffer redBuffer, blueBuffer, greenBuffer, defaultBuffer;

    private int rainbowFirstPixelHue;

    

    /**
     * @param port the PWM port this strip of LEDs is connected to
     * @param length the number of LED 'pixels' in this strip
     */
    public LEDSubsystem(int port, int length) {

        this.led = new AddressableLED(port);
        this.redBuffer = new AddressableLEDBuffer(length);
        this.greenBuffer = new AddressableLEDBuffer(length);
        this.blueBuffer = new AddressableLEDBuffer(length);
        this.defaultBuffer = new AddressableLEDBuffer(length);

        for (var i = 0; i < redBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            redBuffer.setRGB(i, 255, 0, 0);
         }

         for (var i = 0; i < blueBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            blueBuffer.setRGB(i, 0, 0, 255);
         }

         for (var i = 0; i < greenBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            greenBuffer.setRGB(i, 0, 255, 0);
         }
        
        led.setLength(defaultBuffer.getLength());
        led.setData(defaultBuffer);
        led.start();

        rainbowFirstPixelHue = 0;
    }

    public void periodic() {
        
    }

    private void rainbow() {
        // For every pixel
        for (var i = 0; i < defaultBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (rainbowFirstPixelHue + (i * 180 / defaultBuffer.getLength())) % 180;
            // Set the value
            defaultBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue += 3;
        // Check bounds
        rainbowFirstPixelHue %= 180;
    }

    public void setRedBuffer(){
       setBuffer(redBuffer);
    }

    public void setGreenBuffer(){
       setBuffer(greenBuffer);
    }

    public void setBlueBuffer(){
        setBuffer(blueBuffer);
    }

    private void setBuffer(AddressableLEDBuffer buffer){
        led.setData(buffer);
        led.start();
    }
}
