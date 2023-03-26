package frc.team670.mustanglib.utils;

import edu.wpi.first.wpilibj.util.Color;

/**
 * Represents colors that our LED subsystem supports
 * Extends WPILib Color
 * Stores Colors in hsv format
 * 
 */
public class LEDColor {

    

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