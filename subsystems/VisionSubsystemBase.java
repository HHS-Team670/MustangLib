package frc.team670.mustanglib.subsystems;

import java.util.Arrays;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.constants.RobotConstantsBase;


/**
 * Stores values off of NetworkTables for easy retrieval and gives them
 * Listeners to update the stored values as they are changed.
 * 
 * @author Katia Bravo
 */
public abstract class VisionSubsystemBase extends MustangSubsystemBase {

    protected Pair<PhotonCamera, Transform3d>[] camerasAndOffsets;
    private RobotPoseEstimator photonPoseEstimator;
    
    private PowerDistribution pd;
    // protected Pose2d startPose = new Pose2d(0, 0, new Rotation2d(0));

    // protected double distance;
    // protected double angle;
    protected double visionCapTime;
    private boolean hasTarget;

    private boolean ledsTurnedOn;

    private boolean overriden;

    
    public VisionSubsystemBase(PowerDistribution pd, AprilTagFieldLayout visionFieldLayout, Pair<PhotonCamera, Transform3d>... camerasAndOffsets) {
        this.pd = pd;
        this.camerasAndOffsets = camerasAndOffsets;
        photonPoseEstimator = new RobotPoseEstimator(visionFieldLayout, PoseStrategy.AVERAGE_BEST_TARGETS, Arrays.asList(camerasAndOffsets));
    }
    
    public boolean hasTarget() {
        return hasTarget;
    }
    
    /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return A pair of the fused camera observations to a single Pose2d on the field, and the
     *         time of the observation. Assumes a planar field and the robot is always firmly on
     *         the ground
     */
    public Optional<Pair<Pose3d, Double>> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    /**
     * 
     * @return distance, in inches, from the camera to the target
     */
    // protected void processImage(double cameraHeight, double targetHeight, double cameraAngleDeg) {
    //     var result = camera.getLatestResult();
    //     List<PhotonTrackedTarget> targets = result.getTargets();
    //     double lastDistanceCapTime = Math.abs(getVisionCaptureTime() - Timer.getFPGATimestamp());

    //     if (targets.size() > 0) {
    //         hasTarget = true;
    //         PhotonTrackedTarget target = targets.get(0);
    //         angle = target.getYaw();
    //         distance = PhotonUtils.calculateDistanceToTargetMeters(
    //                 cameraHeight, targetHeight,
    //                 Units.degreesToRadians(cameraAngleDeg),
    //                 Units.degreesToRadians(target.getPitch()));
    //         visionCapTime = Timer.getFPGATimestamp() - result.getLatencyMillis() / 1000;
    //     } else if (lastDistanceCapTime > 0.25){
    //         hasTarget = false;
    //         distance = RobotConstantsBase.VISION_ERROR_CODE;
    //         // Logger.consoleLog("NO TARGET DETECTED");
    //     }
    // }

    // public void adjustDistance(double adjustment) {
    //     distance += adjustment;
    // }

    // public double getDistanceToTargetM() {
    //     return hasTarget ? distance : RobotConstantsBase.VISION_ERROR_CODE;
    // }

    public boolean isValidImage(){
        double lastDistanceCapTime = Math.abs(getVisionCaptureTime() - Timer.getFPGATimestamp());
        if(lastDistanceCapTime < 2){
            return true;
        }
        return false;
    }

    // public double getDistanceToTargetCm() {
    //     return getDistanceToTargetM() * 100;
    // }

    // public double getDistanceToTargetInches() {
    //     return getDistanceToTargetM() * 100 / 2.54;
    // }

    // public double getAngleToTarget() {
    //     return hasTarget ? angle : RobotConstantsBase.VISION_ERROR_CODE;
    // }

    // public VisionMeasurement getPoseVisionMeasurements(double heading, Pose2d targetPose, Pose2d cameraOffset) {
    //     // specific to fixed target point from a single side
    //     if (hasTarget()){
    //         Translation2d camToTargetTranslation = PhotonUtils.estimateCameraToTargetTranslation(distance, Rotation2d.fromDegrees(angle));
    //         Transform2d camToTargetTrans = PhotonUtils.estimateCameraToTarget(camToTargetTranslation, targetPose, Rotation2d.fromDegrees(heading));
    //         Pose2d targetOffset = cameraOffset.transformBy(camToTargetTrans.inverse());
    //         return new VisionMeasurement(targetOffset, visionCapTime);
    //     }
    //     return null;
    // }

    // public void setStartPoseDeg(double x, double y, double angle) {
    //     startPose = new Pose2d(x, y, Rotation2d.fromDegrees(angle));
    // }

    // public void setStartPoseRad(double x, double y, double angle) {
    //     startPose = new Pose2d(x, y, new Rotation2d(angle));
    // }

    public double getVisionCaptureTime() {
        return visionCapTime;
    }

    public void switchLEDS(boolean on, boolean override) {
        pd.setSwitchableChannel(on);
        ledsTurnedOn = on;
        overriden = override;
    }

    public void switchLEDS(boolean on){
        switchLEDS(on, false);
    }

    public boolean LEDSOverriden(){
        return overriden;
    }

    public boolean LEDsTurnedOn() {
        return ledsTurnedOn;
    }

    public void testLEDS() {
        pd.setSwitchableChannel(SmartDashboard.getBoolean("LEDs on", true));
    }

    @Override
    public HealthState checkHealth() {
        return HealthState.GREEN;
    }

    public class VisionMeasurement {
        public Pose2d pose;
        public double capTime;

        public VisionMeasurement(Pose2d pose, double capTime) {
            this.capTime = capTime;
            this.pose = pose;
        }
    }

    // public PhotonCamera getCamera(){
    //     return camera;
    // }

    public Pair<PhotonCamera, Transform3d>[] getCamerasAndOffsets(){
        return camerasAndOffsets;
    }

    public PhotonCamera[] getCameras() {
        PhotonCamera[] cameras = new PhotonCamera[camerasAndOffsets.length];
        for (int i = 0; i < cameras.length; i++) {
            cameras[i] = camerasAndOffsets[i].getFirst();
        }
        return cameras;
    }



}