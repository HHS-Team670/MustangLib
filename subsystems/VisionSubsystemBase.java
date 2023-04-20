package frc.team670.mustanglib.subsystems;

import java.util.Optional;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadPoolExecutor;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.team670.mustanglib.utils.Logger;


/**
 * Subsystem base vision. Mainly used for april tags pose estimation.
 * 
 * @author ethan c
 */
public abstract class VisionSubsystemBase extends MustangSubsystemBase {
    protected PhotonCameraWrapper[] mCameras;
    private AprilTagFieldLayout mVisionFieldLayout;
    private ThreadPoolExecutor mExecutor = (ThreadPoolExecutor) Executors.newFixedThreadPool(5);
    private ConcurrentLinkedQueue<VisionMeasurement> mVisionMeasurementsBuffer;
    private boolean mInit = false;

    private PhotonCamera[] cams;
    private Transform3d[] cameraOffsets;

    public VisionSubsystemBase(AprilTagFieldLayout visionFieldLayout, PhotonCamera[] cams,
            Transform3d[] cameraOffsets) {
        this.cams = cams;
        this.mVisionFieldLayout = visionFieldLayout;
        this.cameraOffsets = cameraOffsets;
    }

    /**
     * DO NOT CALL IN ROBOT INIT! DS IS NOT NECESSARILY READY THEN. CALL IN PERIODIC OR AUTONINIT.
     * More details here: https://www.chiefdelphi.com/t/getalliance-always-returning-red/425782/27
     */
    public void initalize() {
        // does nothing if DS not initialized yet
        if (DriverStation.getAlliance() == Alliance.Invalid) {
            mInit = false;
            return;
        }
        setFieldOrigin();

        PhotonCameraWrapper[] c = new PhotonCameraWrapper[cams.length];
        for (int i = 0; i < cams.length; i++) {
            c[i] = new PhotonCameraWrapper(cams[i], cameraOffsets[i], mVisionFieldLayout);
        }
        this.mCameras = c;
        mInit = true;
    }

    private void setFieldOrigin() {
        var origin = DriverStation.getAlliance() == Alliance.Blue
                ? OriginPosition.kBlueAllianceWallRightSide
                : OriginPosition.kRedAllianceWallRightSide;
        mVisionFieldLayout.setOrigin(origin);
    }

    @Override
    public void mustangPeriodic() {
        if (!mInit) {
            initalize();
        } else {
            mExecutor.submit(
                () -> {
                    processVisionMeasurements();
                }
            );
        }
    }

    public boolean hasTarget() {
        for (PhotonCameraWrapper pcw : mCameras) {
            var targets = pcw.getCamera().getLatestResult().targets;
            if (targets.isEmpty())
                return false;
        }
        return true;
    }

    public boolean isInitialized() {
        return mInit;
    }

    public record VisionMeasurement(EstimatedRobotPose kEstimatedRobotPose, double kXConfidence,
            double kYConfidence, double kThetaConfidence) {
    }

    public VisionMeasurement getVisionMeasurement() {
        return mVisionMeasurementsBuffer.poll();
    }

    private void processVisionMeasurements() {

    }

    /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to
     *         create the estimate
     */
    public EstimatedRobotPose[] getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (!mInit) {
            Logger.consoleLog("Vision not initalized!", this);
            return null;
        }

        EstimatedRobotPose[] poses = new EstimatedRobotPose[mCameras.length];
        for (int i = 0; i < poses.length; i++) {
            var bestTarget = mCameras[i].getCamera().getLatestResult().getBestTarget();
            if (bestTarget != null) {
                if (bestTarget.getPoseAmbiguity() > 1.5) {
                    poses[i] = null;
                } else {
                    poses[i] =
                            mCameras[i].getEstimatedGlobalPose(prevEstimatedRobotPose).orElse(null);
                }

                poses[i] = mCameras[i].getEstimatedGlobalPose(prevEstimatedRobotPose).orElse(null);
            } else {
                poses[i] = null;
            }

        }
        return poses;

    }
    // public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    // double avgX, avgY, avgDeg, avgTime;
    // avgX = avgY = avgDeg = avgTime = 0;
    // for (int i = 0; i < cameras.length; i++) {
    // EstimatedRobotPose p =
    // cameras[i].getEstimatedGlobalPose(prevEstimatedRobotPose).orElse(null);
    // if (p == null) return null;

    // avgX += p.estimatedPose.toPose2d().getX();
    // avgY += p.estimatedPose.toPose2d().getX();
    // avgDeg += p.estimatedPose.toPose2d().getRotation().getDegrees();
    // avgTime += p.timestampSeconds;
    // }
    // avgX /= cameras.length;
    // avgY /= cameras.length;
    // avgDeg /= cameras.length;
    // avgTime /= cameras.length;

    // return new Pair<>(new Pose2d(avgX, avgY, new Rotation2d(avgDeg)), avgTime);
    // }

    @Override
    public HealthState checkHealth() {
        return HealthState.GREEN;
    }

    public PhotonCamera[] getmCameras() {
        return this.cams;
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

}
