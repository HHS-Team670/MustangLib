package frc.team670.mustanglib.subsystems;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.utils.Logger;


/**
 * Stores values off of NetworkTables for easy retrieval and gives them Listeners to update the
 * stored values as they are changed.
 */
public abstract class VisionSubsystemBase extends MustangSubsystemBase {

    private PowerDistribution pd;
    protected PhotonCameraWrapper[] cameras;

    private PhotonCamera[] cams;
    private Transform3d[] cameraOffsets;
    private AprilTagFieldLayout visionFieldLayout;
    // protected double visionCapTime;
    // private boolean hasTarget;
    // private AprilTagFieldLayout visionFieldLayout;
    private boolean ledsTurnedOn;
    private boolean overridden;
    private boolean init = false;



    public VisionSubsystemBase(PowerDistribution pd, AprilTagFieldLayout visionFieldLayout,
            PhotonCamera[] cams, Transform3d[] cameraOffsets) {
        this.pd = pd;
        this.cams = cams;
        this.visionFieldLayout = visionFieldLayout;
        this.cameraOffsets = cameraOffsets;
    }

    /**
     * DO NOT CALL IN ROBOT INIT! DS IS NOT NECESSARILY READY THEN. CALL IN PERIODIC OR AUTONINIT.
     * More details here: https://www.chiefdelphi.com/t/getalliance-always-returning-red/425782/27
     */
    public void initalize() {
        // SmartDashboard.putString("VISION INIT: DRIVER STATION ALLIANCE:", "" +
        // DriverStation.getAlliance());
        // does nothing if DS not initialized yet
        if (DriverStation.getAlliance() == Alliance.Invalid) {
            init = false;
            return;
        }

        var origin = DriverStation.getAlliance() == Alliance.Blue
                ? OriginPosition.kBlueAllianceWallRightSide
                : OriginPosition.kRedAllianceWallRightSide;
        visionFieldLayout.setOrigin(origin);

        PhotonCameraWrapper[] c = new PhotonCameraWrapper[cams.length];
        for (int i = 0; i < cams.length; i++) {
            c[i] = new PhotonCameraWrapper(cams[i], cameraOffsets[i], visionFieldLayout);
        }
        this.cameras = c;
        init = true;
    }

    public void setAprilTagFieldLayout(AprilTagFieldLayout field) {
        this.visionFieldLayout = field;
        var origin = DriverStation.getAlliance() == Alliance.Blue
                ? OriginPosition.kBlueAllianceWallRightSide
                : OriginPosition.kRedAllianceWallRightSide;
        visionFieldLayout.setOrigin(origin);
        // rest cams with new field
        PhotonCameraWrapper[] c = new PhotonCameraWrapper[cams.length];
        for (int i = 0; i < cams.length; i++) {
            c[i] = new PhotonCameraWrapper(cams[i], cameraOffsets[i], visionFieldLayout);
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

    public boolean isInitialized() {
        return init;
    }

    /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return an EstimatedRobotPose with an estimated pose, the timestamp, and targets used to
     *         create the estimate
     */
    public EstimatedRobotPose[] getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        if (!init) {
            Logger.consoleLog("Vision not initalized!", this);
            return null;
        }

        EstimatedRobotPose[] poses = new EstimatedRobotPose[cameras.length];
        for (int i = 0; i < poses.length; i++) {
            var bestTarget = cameras[i].getCamera().getLatestResult().getBestTarget();
            if (bestTarget != null) {
                // if (bestTarget.getPoseAmbiguity() > 0.15) {
                //     poses[i] = null;
                // } else {
                //     poses[i] = cameras[i].getEstimatedGlobalPose(prevEstimatedRobotPose).orElse(null);
                // }

                poses[i] = cameras[i].getEstimatedGlobalPose(prevEstimatedRobotPose).orElse(null);
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

    public void switchLEDS(boolean on, boolean override) {
        pd.setSwitchableChannel(on);
        ledsTurnedOn = on;
        overridden = override;
    }

    public void switchLEDS(boolean on) {
        switchLEDS(on, false);
    }

    public boolean LEDSOverridden() {
        return overridden;
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
