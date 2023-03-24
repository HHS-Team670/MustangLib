package frc.team670.mustanglib.utils;

import edu.wpi.first.wpilibj.util.Color;

/**
 * Represents colors that our LED subsystem supports
 * Extends WPILib Color
 * Stores Colors in rgb format
 * 
 */
public class LEDColor extends Color {

    public static final LEDColor BLUE = new LEDColor(0, 0, 255);

    public static final LEDColor GREEN = new LEDColor(25, 185, 22);// rgb

    public static final LEDColor RED = new LEDColor(255, 0, 0);// rgb

    public static final LEDColor LIGHT_BLUE = new LEDColor(4, 0, 255);// hsv (actually green)

    public static final LEDColor SEXY_PURPLE = new LEDColor(37, 4, 61); // rgb

    public static final LEDColor SEXY_YELLOW = new LEDColor(250, 60, 0); // rgb

    public static final LEDColor WHITE = new LEDColor(255, 255, 255);// rgb

    public LEDColor(double red, double green, double blue) {
        super(red, green, blue);
    }

    public LEDColor dimmer() {
        return new LEDColor(red / 1.25, blue / 1.25, green / 1.25);
    }

    public LEDColor brighter() {
        return new LEDColor(red * 1.25, blue * 1.25, green * 1.25);
    }
}