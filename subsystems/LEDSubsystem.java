package frc.team670.mustanglib.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public abstract class LEDSubsystem extends MustangSubsystemBase {
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private int length;

    private int m_rainbowFirstPixelHue;

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
    }

    @Override
    public void mustangPeriodic() {
        m_led.setData(m_ledBuffer);
    }

    @Override
    public void debugSubsystem() {

    }

    @Override
    public HealthState checkHealth() {
        return HealthState.GREEN;
    }

    public static double[] RGBtoHSV(double r, double g, double b) {

        double h, s, v;

        double min, max, delta;

        min = Math.min(Math.min(r, g), b);
        max = Math.max(Math.max(r, g), b);

        // V
        v = max;

        delta = max - min;

        // S
        if (max != 0)
            s = delta / max;
        else {
            s = 0;
            h = -1;
            return new double[] { h, s, v };
        }

        // H
        if (r == max)
            h = (g - b) / delta; // between yellow & magenta
        else if (g == max)
            h = 2 + (b - r) / delta; // between cyan & yellow
        else
            h = 4 + (r - g) / delta; // between magenta & cyan

        h *= 60; // degrees

        if (h < 0)
            h += 360;

        return new double[] { h, s, v };
    }

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

    public void solid(Color c){
        // For every pixel
        double[] hsv = RGBtoHSV(c.red, c.green, c.blue);
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Set the value
            m_ledBuffer.setHSV(i, (int)hsv[0], (int)hsv[1], (int)hsv[2]);
        }
    }

    public void resourceBar(Color c, double ratioFilled){
        // For every pixel
        double[] hsv = RGBtoHSV(c.red, c.green, c.blue);
        int ratioBright = (int)((double)m_ledBuffer.getLength() * ratioFilled);
        for (var i = 0; i < ratioBright; i++) {
            // Set the value
            m_ledBuffer.setHSV(i, (int)hsv[0], (int)hsv[1], (int)hsv[2]);
        }
        for (var i = ratioBright; i < m_ledBuffer.getLength(); i++) {
            // Set the value
            m_ledBuffer.setHSV(i, (int)hsv[0], (int)hsv[1], (int)(hsv[2]/2));
        }
    }

    public void setBuffer(AddressableLEDBuffer buffer){
        this.m_ledBuffer = buffer;
    }

}
