package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    private int rainbowFirstPixelHue;

    /**
     * @param port the PWM port this strip of LEDs is connected to
     * @param length the number of LED 'pixels' in this strip
     */
    public LEDSubsystem(int port, int length) {

        this.led = new AddressableLED(port);
        this.ledBuffer = new AddressableLEDBuffer(length);

        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);
        led.start();

        rainbowFirstPixelHue = 0;
    }

    public void periodic() {
        // TODO: set different animations/colors based on conditions
        led.setData(ledBuffer);
        rainbow();

    }

    private void rainbow() {
        // For every pixel
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
            // Set the value
            ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue += 3;
        // Check bounds
        rainbowFirstPixelHue %= 180;
    }
}
