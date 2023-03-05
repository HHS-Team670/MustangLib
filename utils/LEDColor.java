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
    public static final LEDColor LIGHT_BLUE = new LEDColor(241);

    public static final LEDColor SEXY_PURPLE = new LEDColor(100, 15, 130); // 39, 26, 92
   // public static final LEDColor SEXi_PURPLE = new LEDColor(82, 17, 112);

    //public static final LEDColor YELLOW = new LEDColor(255, 225, 0); //280, 210, 1 sexy green - kedar
    
    //public static final LEDColor CONE = new LEDColor(250, 80, 0);
    public static final LEDColor SEXY_YELLOW = new LEDColor(250, 60, 0);

    public static final LEDColor WHITE = new LEDColor(255,255,255);

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