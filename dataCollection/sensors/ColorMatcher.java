package frc.team670.mustanglib.dataCollection.sensors;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class ColorMatcher {
  /**
   * Change the I2C port below to match the connection of your color sensor
   */
  private final I2C.Port i2cPort = I2C.Port.kMXP;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a parameter.
   * The device will be automatically initialized with default parameters.
   */
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  /**
   * A Rev Color Match object is used to register and detect known colors. This
   * can be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  private final ColorMatch m_colorMatcher = new ColorMatch();

  public enum colors {

    BLUE(0, ColorMatch.makeColor(0.136, 0.412, 0.450)), 
    YELLOW(1, ColorMatch.makeColor(0.293, 0.561, 0.144)),
    RED(2, ColorMatch.makeColor(0.475, 0.371, 0.153)), 
    GREEN(3, ColorMatch.makeColor(0.196, 0.557, 0.246));

    private int colorNumber;
    private Color color;

    private colors(int colorNumber, Color color) {
      this.colorNumber = colorNumber;
      this.color = color;
    }

    /**
     * 
     * @return the corresponding integer code for each color on the wheel
     */
    public int getColorNumber() {
      return colorNumber;
    }

    private Color getTargetColor() {
      return color;
    }
  }

  public static final int UNKNOWN_COLOR_NUMBER = -1;

  private final double CONFIDENCE_THRESHOLD = 0.85;

  public ColorMatcher() {
    init();
  }

  public void init() {
    m_colorMatcher.addColorMatch(colors.BLUE.getTargetColor());
    m_colorMatcher.addColorMatch(colors.YELLOW.getTargetColor());
    m_colorMatcher.addColorMatch(colors.RED.getTargetColor());
    m_colorMatcher.addColorMatch(colors.GREEN.getTargetColor());

    m_colorMatcher.setConfidenceThreshold(CONFIDENCE_THRESHOLD);
  }

  public int detectColor() {
    /**
     * The method GetColor() returns a normalized color value from the sensor and
     * can be useful if outputting the color to an RGB LED or similar. To read the
     * raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in well
     * lit conditions (the built in LED is a big help here!). The farther an object
     * is the more light from the surroundings will bleed into the measurements and
     * make it difficult to accurately determine its color.
     */
    Color detectedColor = m_colorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
    // String colorString;
    int colorNumber;

    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == colors.BLUE.getTargetColor()) {
      // colorString = "Blue";
      colorNumber = colors.BLUE.getColorNumber();
    } else if (match.color == colors.YELLOW.getTargetColor()) {
      // colorString = "Yellow";
      colorNumber = colors.YELLOW.getColorNumber();
    } else if (match.color == colors.RED.getTargetColor()) {
      // colorString = "Red";
      colorNumber = colors.RED.getColorNumber();
    } else if (match.color == colors.GREEN.getTargetColor()) {
      // colorString = "Green";
      colorNumber = colors.GREEN.getColorNumber();
    } else {
      // colorString = "Unknown";
      colorNumber = UNKNOWN_COLOR_NUMBER;
    }

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the sensor.
     */
    /*
     * SmartDashboard.putNumber("Red", detectedColor.red);
     * SmartDashboard.putNumber("Green", detectedColor.green);
     * SmartDashboard.putNumber("Blue", detectedColor.blue);
     * SmartDashboard.putNumber("Confidence", match.confidence);
     * SmartDashboard.putString("Detected Color", colorString);
     * SmartDashboard.putNumber("Detected Color Number", colorNumber);
     */

    return colorNumber;
  }
}