package frc.team670.mustanglib.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.constants.RobotConstants;


/**
 * Stores values off of NetworkTables for easy retrieval and gives them
 * Listeners to update the stored values as they are changed.
 * 
 * @author Katia Bravo
 */
public abstract class VisionSubsystemBase extends MustangSubsystemBase {

    private PhotonCamera camera;
    protected Pose2d startPose = new Pose2d(0, 0, new Rotation2d(0));

    protected double distance;
    protected double angle;
    protected double visionCapTime;
    private boolean hasTarget;

    private PowerDistribution pd;

    public VisionSubsystemBase(PowerDistribution pd){
        this.pd = pd;
    }

    public void setCameraName(String cameraName) {
        camera = new PhotonCamera(cameraName);
    }

    public boolean hasTarget(){
        return hasTarget;
    }

    /**
     * 
     * @return distance, in inches, from the camera to the target
     */
    public void processImage(double cameraHeight, double targetHeight, double cameraAngleDeg) {
        var result = camera.getLatestResult();

        if(result.hasTargets()){
            hasTarget = true;
            angle = result.getTargets().get(0).getYaw();
            distance = PhotonUtils.calculateDistanceToTargetMeters(
                    cameraHeight, targetHeight,
                    Units.degreesToRadians(cameraAngleDeg),
                    Units.degreesToRadians(result.getBestTarget().getPitch()));                
            visionCapTime = Timer.getFPGATimestamp() - result.getLatencyMillis() / 1000;
        } else {
            hasTarget = false;
            // Logger.consoleLog("NO TARGET DETECTED");
        }
    }

    public void adjustDistance(double adjustment) {
        distance += adjustment;
    }

    public double getDistanceToTargetM() {
        return hasTarget ? distance : RobotConstants.VISION_ERROR_CODE;
    }

    public double getDistanceToTargetCm() {
        return getDistanceToTargetM() * 100;
    }

    public double getDistanceToTargetInches() {
        return getDistanceToTargetM() * 100 / 2.54;
    }

    public double getAngleToTarget() {
        return hasTarget ? angle : RobotConstants.VISION_ERROR_CODE;
    }

    public abstract VisionMeasurement getVisionMeasurements(double heading, Pose2d targetPose, Pose2d cameraOffset);

    public void setStartPoseRad(double x, double y, double angle) {
        startPose = new Pose2d(x, y, new Rotation2d(angle));
    }

    public void setStartPoseDeg(double x, double y, double degrees) {
        startPose = new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
    }

    public double getVisionCaptureTime() {
        return visionCapTime;
    }

    public void switchLEDs(boolean on) {
        pd.setSwitchableChannel(on);
    }

    public boolean LEDsTurnedOn(){
        return pd.getSwitchableChannel();
    }

    public void testLEDS() {
        pd.setSwitchableChannel(SmartDashboard.getBoolean("LEDs on", true));
    }

    @Override
    public HealthState checkHealth() {
        return HealthState.RED;
    }

    public class VisionMeasurement{
        public Pose2d pose;
        public double capTime;

        public VisionMeasurement(Pose2d pose, double capTime){
            this.capTime = capTime;
            this.pose = pose;
        }
    }

}