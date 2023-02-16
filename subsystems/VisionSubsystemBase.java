package frc.team670.mustanglib.subsystems;

import java.util.Optional;
import org.opencv.aruco.EstimateParameters;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.constants.RobotConstants;


/**
 * Stores values off of NetworkTables for easy retrieval and gives them Listeners to update the
 * stored values as they are changed.
 */
public abstract class VisionSubsystemBase extends MustangSubsystemBase {

    protected PhotonCameraWrapper[] cameras;
    private PowerDistribution pd;
    // protected double visionCapTime;
    // private boolean hasTarget;

    private boolean ledsTurnedOn;

    private boolean overriden;


    public VisionSubsystemBase(PowerDistribution pd, AprilTagFieldLayout visionFieldLayout,
            PhotonCamera[] cameras, Transform3d[] cameraOffsets) {
        this.pd = pd;
        PhotonCameraWrapper[] cams = new PhotonCameraWrapper[cameras.length];

        for (int i = 0; i < cameras.length; i++) {
            cams[i] = new PhotonCameraWrapper(cameras[i], cameraOffsets[i], visionFieldLayout);
        }
    }

    public boolean hasTarget() {
        for (PhotonCameraWrapper pcw : cameras) {
            var targets = pcw.getCamera().getLatestResult().targets;
            if (targets.isEmpty())
                return false;
        }
        return true;
    }

    /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to
     *         create the estimate
     */
    public EstimatedRobotPose[] getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        EstimatedRobotPose[] poses = new EstimatedRobotPose[cameras.length];
        for (int i = 0; i < poses.length; i++) 
            poses[i] = cameras[i].getEstimatedGlobalPose(prevEstimatedRobotPose).orElse(null);
        
        return poses;
    }

    public void switchLEDS(boolean on, boolean override) {
        pd.setSwitchableChannel(on);
        ledsTurnedOn = on;
        overriden = override;
    }

    public void switchLEDS(boolean on) {
        switchLEDS(on, false);
    }

    public boolean LEDSOverriden() {
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

    public PhotonCamera[] getCameras() {
        PhotonCamera[] cameras = new PhotonCamera[this.cameras.length];
        for (int i = 0; i < cameras.length; i++) {
            cameras[i] = this.cameras[i].getCamera();
        }
        return cameras;
    }

    private class PhotonCameraWrapper {
        private PhotonCamera photonCamera;
        private PhotonPoseEstimator photonPoseEstimator;

        public PhotonCameraWrapper(PhotonCamera photonCamera, Transform3d robotToCam,
                AprilTagFieldLayout fieldLayout) {
            this.photonCamera = photonCamera;

            photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP,
                    photonCamera, robotToCam);
            photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }

        /**
         * @param estimatedRobotPose The current best guess at robot pose
         * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to
         *         create the estimate
         */
        public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
            if (photonPoseEstimator == null) {
                // The field layout failed to load, so we cannot estimate poses.
                return Optional.empty();
            }
            photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
            return photonPoseEstimator.update();
        }

        public PhotonCamera getCamera() {
            return photonCamera;
        }
    }

    // public boolean isValidImage() {
    // double lastDistanceCapTime = Math.abs(getVisionCaptureTime() - Timer.getFPGATimestamp());
    // if (lastDistanceCapTime < 2) {
    // return true;
    // }
    // return false;
    // }
    // public double getVisionCaptureTime() {
    // return visionCapTime;
    // }
    // public class VisionMeasurement {
    // public Pose2d pose;
    // public double capTime;

    // public VisionMeasurement(Pose2d pose, double capTime) {
    // this.capTime = capTime;
    // this.pose = pose;
    // }
    // }
    // public double getDistanceToTargetCm() {
    // return getDistanceToTargetM() * 100;
    // }

    // public double getDistanceToTargetInches() {
    // return getDistanceToTargetM() * 100 / 2.54;
    // }

    // public double getAngleToTarget() {
    // return hasTarget ? angle : RobotConstantsBase.VISION_ERROR_CODE;
    // }

    // public VisionMeasurement getPoseVisionMeasurements(double heading, Pose2d targetPose, Pose2d
    // cameraOffset) {
    // // specific to fixed target point from a single side
    // if (hasTarget()){
    // Translation2d camToTargetTranslation =
    // PhotonUtils.estimateCameraToTargetTranslation(distance, Rotation2d.fromDegrees(angle));
    // Transform2d camToTargetTrans = PhotonUtils.estimateCameraToTarget(camToTargetTranslation,
    // targetPose, Rotation2d.fromDegrees(heading));
    // Pose2d targetOffset = cameraOffset.transformBy(camToTargetTrans.inverse());
    // return new VisionMeasurement(targetOffset, visionCapTime);
    // }
    // return null;
    // }

    // public void setStartPoseDeg(double x, double y, double angle) {
    // startPose = new Pose2d(x, y, Rotation2d.fromDegrees(angle));
    // }

    // public void setStartPoseRad(double x, double y, double angle) {
    // startPose = new Pose2d(x, y, new Rotation2d(angle));
    // }

    // public PhotonCamera getCamera(){
    // return camera;
    // }

    // public Pair<PhotonCamera, Transform3d>[] getCamerasAndOffsets(){
    // return camerasAndOffsets;
    // }

    // public PhotonCamera[] getCameras() {
    // PhotonCamera[] cameras = new PhotonCamera[camerasAndOffsets.length];
    // for (int i = 0; i < cameras.length; i++) {
    // cameras[i] = camerasAndOffsets[i].getFirst();
    // }
    // return cameras;
    // }

}
