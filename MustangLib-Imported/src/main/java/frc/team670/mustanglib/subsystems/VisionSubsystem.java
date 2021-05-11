package frc.team670.mustanglib.subsystems;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team670.mustanglib.constants.RobotConstants;
import frc.team670.mustanglib.utils.MustangNotifications;

/**
 * Stores values off of NetworkTables for easy retrieval and gives them
 * Listeners to update the stored values as they are changed.
 * 
 * @author lakshbhambhani, katia, pallavi
 */
public class VisionSubsystem extends MustangSubsystemBase {

    private Solenoid cameraLEDs;

    private final String VISION_TRIGGER_KEY = "vision-enable";
    private final String VISION_VALUES_KEY = "vision-values";

    private final String VISION_MIN_HSV_KEY = "vision-minHSV";
    private final String VISION_MAX_HSV_KEY = "vision-maxHSV";
    private final String VISION_SHAPE_KEY = "vision-shape";

    private final String VISION_HEALTH_KEY = "vision-health";

    private double angle;
    private double distance;

    /**
     * Used to create a mustangCoprocessor object based on the key of the table that returns the values from vision processing
     */
    private VisionSubsystem(String key, double[] minHSV, double[] maxHSV, double[] shape, int PCMModulePort, int visionLEDPCMPort) {
        cameraLEDs = new Solenoid(PCMModulePort, visionLEDPCMPort);
        SmartDashboard.putBoolean("LEDs on", false);
        
        SmartDashboard.putNumberArray(VISION_MIN_HSV_KEY, minHSV);
        SmartDashboard.putNumberArray(VISION_MAX_HSV_KEY, maxHSV);
        SmartDashboard.putNumberArray(VISION_SHAPE_KEY, shape);
    }

    /**
     * Used to trigger the vision system to run and get new values
     */
    public void triggerVision() {
        SmartDashboard.putBoolean(VISION_TRIGGER_KEY, true);
    }

    /**
     * Used to trigger the vision system to run and get new values
     */
    public void getLatestVisionData() {
        Double[] values = SmartDashboard.getNumberArray(VISION_VALUES_KEY, new Double[] {-1.0,-1.0});
        angle = values[0];
        distance = values[1];
    }

    /**
     * Used to clear last values present on the table
     */
    public void clearLastValues(){
        SmartDashboard.putNumberArray(VISION_VALUES_KEY, new Double[] {-1.0,-1.0});
    }

    /**
     * 
     * @return the horizontal angle between the camera-forward to the robot-target line
     */
    public double getAngleToTarget() {
        getLatestVisionData();
        return angle;
    }

    /**
     * 
     * @return distance, in inches, from the camera to the target
     */
    public double getDistanceToTargetInches() {
        getLatestVisionData();
        return distance;
    }

    /**
     * 
     * @return distance, in cm, from the camera to the target
     */
    public double getDistanceToTargetCm() {
        return getDistanceToTargetInches() * 2.54;
    }

    /**
     * Used to turn on the bright green leds for vision
     */
    public void turnOnLEDs() {
        cameraLEDs.set(true);
    }

    /**
     * Used to turn off the bright green leds for vision
     */
    public void turnOffLEDs() {
        cameraLEDs.set(false);
    }

    /**
     * Used to test the leds for vision
     */
    public void testLEDS() {
        cameraLEDs.set(SmartDashboard.getBoolean("LEDs on", true));
    }

    @Override
    public HealthState checkHealth() {
        HealthState state;

        String visionHealth = SmartDashboard.getString(VISION_HEALTH_KEY, "RED");

        if (visionHealth == "RED") {
            state = HealthState.RED;
            MustangNotifications.reportError("RED Error: Vision System");
        } else if (visionHealth == "YELLOW") {
            state = HealthState.YELLOW;
            MustangNotifications.reportWarning("YELLOW Error: Vision System");
        } else {
            state = HealthState.GREEN;
        }
        return state;
    }

    @Override
    public void mustangPeriodic() { 

    }

}