package frc.team670.mustanglib.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.mustanglib.utils.MustangNotifications;

/**
 * Stores values off of NetworkTables for easy retrieval and gives them
 * Listeners to update the stored values as they are changed.
 * 
 * @author lakshbhambhani, katia, pallavi
 */
public class VisionSubsystemBase extends MustangSubsystemBase {

    private Solenoid cameraLEDs;

    private final String VISION_TRIGGER_KEY = "vision-enabled";
    
    public final static String VISION_VALUES_BASE_KEY = "vision-values-";
    public final static String[] VISION_SUB_KEY = new String[] {"angle", "distance"};

    private final String VISION_MIN_HSV_BASE_KEY = "vision-min-";
    private final String VISION_MAX_HSV_BASE_KEY = "vision-max-";
    private final String[] VISION_HSV_SUB_KEY = new String[] {"h", "s", "v"};
    
    private final String VISION_HEALTH_KEY = "vision-health";

    private double angle;
    private double distance;

    /**
     * Used to create a mustangCoprocessor object based on the key of the table that returns the values from vision processing
     */
    public VisionSubsystemBase(double[] minHSV, double[] maxHSV, int PCMModulePort, int visionLEDPCMPort, VisionShapePointList shapePointList) {
        cameraLEDs = PCMModulePort == -1 ? null : new Solenoid(PCMModulePort, visionLEDPCMPort);
        SmartDashboard.putBoolean("LEDs on", false);
        
        for(int i = 0; i < VISION_HSV_SUB_KEY.length; i++) {
            SmartDashboard.putNumber(VISION_MIN_HSV_BASE_KEY + VISION_HSV_SUB_KEY[i], minHSV[i]);
            SmartDashboard.putNumber(VISION_MAX_HSV_BASE_KEY + VISION_HSV_SUB_KEY[i], maxHSV[i]);
        }

        // for(int i = 0; i < ds.length; i++) {
        //     SmartDashboard.putNumber(VISION_SHAPE_BASE_KEY + i + VISION_SHAPE_SUB_KEY[0], -1);
        //     SmartDashboard.putNumber(VISION_SHAPE_BASE_KEY + i + VISION_SHAPE_SUB_KEY[1], -1);
        //     SmartDashboard.putNumber(VISION_SHAPE_BASE_KEY + i + VISION_SHAPE_SUB_KEY[2],  -1);
        // }

    }

    /**
     * Used to create a mustangCoprocessor object based on the key of the table that returns the values from vision processing
     */
    public VisionSubsystemBase(double[] minHSV, double[] maxHSV, VisionShapePointList shapePointList) {
        this(minHSV, maxHSV, -1, -1, shapePointList);

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
        // Double[] values = SmartDashboard.getNumberArray(VISION_VALUES_KEY, new Double[] {-1.0,-1.0});
        // angle = values[0];
        // distance = values[1];
        angle = SmartDashboard.getNumber(VISION_VALUES_BASE_KEY + VISION_SUB_KEY[0], -1);
        distance = SmartDashboard.getNumber(VISION_VALUES_BASE_KEY + VISION_SUB_KEY[1], -1);
    }

    /**
     * Used to clear last values present on the table
     */
    public void clearLastValues(){
        for(int i = 0; i < VISION_SUB_KEY.length; i++) {
            SmartDashboard.putNumber(VISION_VALUES_BASE_KEY + VISION_SUB_KEY[i], -1);
        }
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
    private double getDistanceToTargetInches() {
        getLatestVisionData();
        return distance;
    }

    /**
     * 
     * @return distance, in cm, from the camera to the target
     */
    public double getDistanceToTargetMeters() {
        return getDistanceToTargetInches() * 25.4;
    }

    /**
     * Used to turn on the bright green leds for vision
     */
    public void turnOnLEDs() {
        if(cameraLEDs != null){
            cameraLEDs.set(true);
        }
    }

    /**
     * Used to turn off the bright green leds for vision
     */
    public void turnOffLEDs() {
        if(cameraLEDs != null){
            cameraLEDs.set(false);
        }
    }

    /**
     * Used to test the leds for vision
     */
    public void testLEDS() {
        if(cameraLEDs != null){
            cameraLEDs.set(SmartDashboard.getBoolean("LEDs on", true));
        }
    }

    @Override
    public HealthState checkHealth() {
        HealthState state;

        String visionHealth = SmartDashboard.getString(VISION_HEALTH_KEY, "GREEN");

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

    public static class VisionShapePoint{

        double x, y, z;
        int pointNum;

        String entryName = "vision-shapepoint-";

        public VisionShapePoint(double x, double y, double z, int pointNum){
            this.x = x;
            this.y = y;
            this.z = z;
            this.pointNum = pointNum;
        }

        public void pushToSmartDashboard(){
            SmartDashboard.putNumber(entryName + pointNum + "-X", x);
            SmartDashboard.putNumber(entryName + pointNum + "-Y", y);
            SmartDashboard.putNumber(entryName + pointNum + "-Z", z);
        }

        public String getPointName(){
            return entryName + pointNum;
        }
    }

    public static class VisionShapePointList{

        ArrayList<String> shapePointNames = new ArrayList<String>();

        public VisionShapePointList(VisionShapePoint... shapePoints){
            for(VisionShapePoint shapePoint : shapePoints){
                shapePointNames.add(shapePoint.getPointName());
                shapePoint.pushToSmartDashboard();
            }
            String[] array = new String[shapePointNames.size()];
            array = shapePointNames.toArray(array);
            SmartDashboard.putStringArray("vision-shapepoints", array);        
        }
    }

}