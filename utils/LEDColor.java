package frc.team670.mustanglib.utils;

/**
 * Represents colors that our LED subsystem supports
 * Uses Hue, Saturation, and Value (HSV) insteadof RGB
 * 
 */
public class LEDColor {
    public static final LEDColor BLUE = new LEDColor(120); //hsv

    public static final LEDColor GREEN = new LEDColor(25, 185, 22);//rgb

    public static final LEDColor RED = new LEDColor(0);//hsv

    public static final LEDColor LIGHT_BLUE = new LEDColor(241);//hsv (actually green)

    public static final LEDColor SEXY_PURPLE = new LEDColor(37, 4, 61); //rgb

    public static final LEDColor SEXY_YELLOW = new LEDColor(250, 60, 0); //rgb

    public static final LEDColor WHITE = new LEDColor(255,255,255);//rgb

    public final int x;
    public final int y;
    public final int z;

    public LEDColor(int x) {
        this.x = x;
        this.y = 255;
        this.z = 255;
    }

    public LEDColor(int x, int y, int z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public LEDColor dimmer() {
        return new LEDColor(x, y, z / 3);
    }

    public LEDColor brighter() {
        return new LEDColor(x, y, z * 3);
    }
}