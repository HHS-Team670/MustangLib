package frc.team670.mustanglib.utils;

/**
 * Represents colors that our LED subsystem supports
 * Uses Hue, Saturation, and Value (HSV) insteadof RGB
 * 
 */
public class LEDColor {
    public static final LEDColor BLUE = new LEDColor(120);
    public static final LEDColor GREEN = new LEDColor(56);
    public static final LEDColor RED = new LEDColor(0);
    public static final LEDColor PURPLE = new LEDColor(241);

    public final int h;
    public final int s;
    public final int v;

    public LEDColor(int h) {
        this.h = h;
        this.s = 255;
        this.v = 255;
    }

    public LEDColor(int h, int s, int v) {
        this.h = h;
        this.s = s;
        this.v = v;
    }

    public LEDColor dimmer() {
        return new LEDColor(h, s, v / 3);
    }

    public LEDColor brighter() {
        return new LEDColor(h, s, v * 3);
    }
}