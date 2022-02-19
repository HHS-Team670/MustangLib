package frc.team670.robot.subsystems;

import java.lang.System.Logger;
import java.util.Timer;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.MustangSubsystemBase;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.constants.RobotConstants;


/**
 * Stores values off of NetworkTables for easy retrieval and gives them
 * Listeners to update the stored values as they are changed.
 * 
 * @author Katia Bravo
 */
public class VisionSubsystemBase extends MustangSubsystemBase{

    private Solenoid cameraLEDs;
    private PhotonCamera camera;
    private Pose2d startPose = new Pose2d(0, 0, new Rotation2d(0));

    private double distance;
    private double angle;
    private double visionCapTime;
    private boolean hasTarget;

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
    private void processImage() {
        try{
            var result = camera.getLatestResult();

            if(result.hasTargets()){
                hasTarget = true;
                angle = camera.getLatestResult().getTargets().get(0).getYaw();
                distance = PhotonUtils.calculateDistanceToTargetMeters(
                        RobotConstants.CAMERA_HEIGHT_METERS,
                        FieldConstants.HIGH_HUB_HEIGHT,
                        Units.degreesToRadians(RobotConstants.CAMERA_ANGLE_DEGREES),
                        Units.degreesToRadians(result.getBestTarget().getPitch()));                
                visionCapTime = Timer.getFPGATimestamp() - result.getLatencyMillis()/1000;
            } else {
                hasTarget = false;
                // Logger.consoleLog("NO TARGET DETECTED");
            }
            
        } catch(Exception e){
            Logger.consoleLog("NT for vision not found %s", e.getStackTrace());
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

    public VisionMeasurement getVisionMeasurements(double heading, Pose2d targetPose, Pose2d cameraOffset) {}

    public void setStartPoseRad(double x, double y, double angle) {
        startPose = new Pose2d(x, y, new Rotation2d(angle));
    }

    public void setStartPoseDeg(double x, double y, double degrees) {
        startPose = new Pose2d(x, y, new Rotation2d.fromDegrees(degrees));
    }

    public double getVisionCaptureTime() {
        return visionCapTime;
    }

    public void setCamerLEDS(int module, PneumaticsModuleType moduleType, int channel) {
        camera = new Solenoid(module, moduleType, channel);
    }

    public void turnOnLEDs() {
        cameraLEDs.set(true);
    }

    public void turnOffLEDs() {
        cameraLEDs.set(false);
    }

    public void LEDSwitch(boolean on) {
        cameraLEDs.set(on);
    }

    public void testLEDS() {
        cameraLEDs.set(SmartDashboard.getBoolean("LEDs on", true));
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