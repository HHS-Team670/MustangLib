package frc.team670.mustanglib.subsystems;

import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadPoolExecutor;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.team670.mustanglib.utils.ConsoleLogger;

/**
 * Subsystem base vision. Mainly used for april tags pose estimation.
 * 
 * @author ethan c
 */
public abstract class VisionSubsystemBase extends MustangSubsystemBase {
    private Config kConfig;
    private AprilTagFieldLayout kFieldLayout;

    protected PhotonCamera[] mCameras;
    protected CameraPoseEstimator[] mCameraEstimators;
    private ThreadPoolExecutor mExecutor = (ThreadPoolExecutor) Executors.newFixedThreadPool(5);
    private ConcurrentLinkedQueue<VisionMeasurement> mVisionMeasurementsBuffer;
    private boolean mInit = false;
    private LED led;

    /**
     * A vision configuration that stores important information about the field and vision subsystem on the robot
     */
    public static record Config(AprilTagFieldLayout kFieldLayout, String[] kCameraIDs,
            Transform3d[] kCameraOffsets, double kPoseAmbiguityCutOff, int kMaxFrameFIDs,
            List<Set<Integer>> kPossibleFIDCombinations, Map<Integer, TagCountDeviation> kVisionStdFromTagsSeen) {
    }

    public VisionSubsystemBase(Config config) {
        this.kConfig = config;
        this.kFieldLayout = config.kFieldLayout;
        this.mVisionMeasurementsBuffer = new ConcurrentLinkedQueue<>();
        this.mCameras = new PhotonCamera[config.kCameraIDs.length];
        for (int i = 0; i < mCameras.length; i++) {
            mCameras[i] = new PhotonCamera(config.kCameraIDs[i]);
        }
    }

    /**
     * Initalizes the vision subsystem.
     * DO NOT CALL IN ROBOT INIT! DS IS NOT NECESSARILY READY THEN. CALL IN PERIODIC
     * OR AUTONINIT.
     * More details here:
     * https://www.chiefdelphi.com/t/getalliance-always-returning-red/425782/27
     */
    public void initalize() {
        // does nothing if DS not initialized yet
        if (!DriverStation.getAlliance().isPresent()) {
            mInit = false;
            return;
        }
        setFieldOrigin();

        CameraPoseEstimator[] c = new CameraPoseEstimator[mCameras.length];
        for (int i = 0; i < c.length; i++) {
            c[i] = new CameraPoseEstimator(mCameras[i], kConfig.kCameraOffsets[i], kFieldLayout);
        }
        led = LED.getInstance();
        this.mCameraEstimators = c;
        mInit = true;
        
    }

    /**
     * Sets origin based on field side (red alliance or blue alliance)
     */
    private void setFieldOrigin() {
        var origin = DriverStation.getAlliance().get() == Alliance.Blue
                ? OriginPosition.kBlueAllianceWallRightSide
                : OriginPosition.kRedAllianceWallRightSide;
        kFieldLayout.setOrigin(origin);
    }
    
    /**
     * Attempts to initalize vision and estimate robot pose after processing the vision feed
     */
    @Override
    public void mustangPeriodic() {
        // attempt initialization until initialized
        if (!mInit) {
            initalize();
            return;
        }

        for (CameraPoseEstimator c : mCameraEstimators) {
            mExecutor.execute(
                    () -> {
                        processVisionFeed(c);
                    });
        }
    }

   
    /**
     * 
     * @return if initialized
     */
    public boolean isInitialized() {
        return mInit;
    }
    /** A representation of the visions pose estimation and its confidence */
    public record VisionMeasurement(EstimatedRobotPose estimation, Vector<N3> confidence) {}

    /**
     * 
     * @return returns the first unprocessed vision measurement
     */
    public VisionMeasurement getVisionMeasurement() {
        return mVisionMeasurementsBuffer.poll();
    }
    
    /**
     * @return if mVisionMeasurementsBuffer is empty
     */
    public boolean isMeasurementBufferEmpty() {
        return mVisionMeasurementsBuffer.isEmpty();
    }

    /**
     * The function processes a vision feed from a camera estimator, calculates the average distance
     * and deviation, and adds the measurement to a queue.
     * 
     * @param cameraEstimator The cameraEstimator parameter is an instance of the CameraPoseEstimator
     * class. It is used to estimate the pose (position and orientation) of the camera based on visual
     * measurements.
     */
    private void processVisionFeed(CameraPoseEstimator cameraEstimator) {
        Optional<CameraPoseEstimator.CameraEstimatorMeasurement> optMeasurement = cameraEstimator.update();
        if (optMeasurement.isEmpty())
            return;
        CameraPoseEstimator.CameraEstimatorMeasurement measurement = optMeasurement.get();
        EstimatedRobotPose estimation = measurement.estimation;
        double avgDistance = getAverageDistance(estimation); // TODO: figure out how vision std work
        Vector<N3> deviation = kConfig.kVisionStdFromTagsSeen.get(MathUtil.clamp(estimation.targetsUsed.size(),
                0,
                kConfig.kVisionStdFromTagsSeen.keySet().size()))
                .computeDeviation(avgDistance);
        mVisionMeasurementsBuffer.add(new VisionMeasurement(estimation, deviation));
    }
    /**
     * Uses a pose estimation to calculate the average distance from the closest camera on the robot to each apriltag target on the field that was used for pose estimation
     * Used to factor in vision trust
     * @param estimation the pose estimation to be used for the average
     * @return the average distance (in meters?)
     */
    private double getAverageDistance(EstimatedRobotPose estimation) {
        double sumDistance = 0;
        for (var target : estimation.targetsUsed) {
            var t3d = target.getBestCameraToTarget();
            sumDistance += Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
        }
        return sumDistance / estimation.targetsUsed.size();
    }

    /** @return the healthstate of the vision subsystem */
    @Override
    public HealthState checkHealth() {
        HealthState state = HealthState.GREEN;
        int counter = 0;
        // checks through the photon cameras and checks if they are null or !connected
        for (PhotonCamera camera: mCameras) {
            if(camera == null || !camera.isConnected()){
                state = HealthState.YELLOW;
                counter++;
                if (camera.getName() == "Arducam_B") {
                    Logger.recordOutput("Arducam_B_Connected", false);
                }
            } else {
                if (camera.getName() == "Arducam_B") {
                    Logger.recordOutput("Arducam_B_Connected", true);
                }
            }
        }
        //iff all of the cameras are null or not connected healthstate = red
        if (counter == mCameras.length && mCameras.length!=0){
            state = HealthState.RED;
        }
        return state;
    }
    
   /**
    * 
    * @return the cameras this subsystem uses
    */
    public PhotonCamera[] getCameras() {
        return mCameras;
    }

    private class CameraPoseEstimator {
        private PhotonCamera photonCamera;
        private PhotonPoseEstimator estimator;
        private double lastTimeStamp;

        public CameraPoseEstimator(PhotonCamera photonCamera, Transform3d robotToCam,
                AprilTagFieldLayout fieldLayout) {
            this.photonCamera = photonCamera;
            estimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    photonCamera, robotToCam);
            estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        }
        /**
         * A record that represents a estimated pose and a photon result from the camera pose estimator
         */
        public static record CameraEstimatorMeasurement(EstimatedRobotPose estimation, PhotonPipelineResult result) {}

        /**
         * Updates the Camera estimator. Should be called periodically
         * 
         * @return An Optional of the Photon pose estimator estimation and pipline
         *         result.
         *         Bad frames are ignored.
         */
        public Optional<CameraEstimatorMeasurement> update() {
            PhotonPipelineResult result = photonCamera.getLatestResult();
            if (ignoreFrame(result))
                return Optional.empty();
                for(int i = 1; i < result.getTargets().size(); i++){
                    if(led.getAllianceColor() == LED.LEDColor.RED && photonCamera.getName().equals("Arducam_B") && result.getTargets().get(i - 1).getFiducialId() == 3 ||  result.getTargets().get(i - 1).getFiducialId() == 4){
                        led.setLedMode(LED.Mode.VISIONON);
                        break;
                    } else if(led.getAllianceColor() == LED.LEDColor.BLUE && photonCamera.getName().equals("Arducam_B") && result.getTargets().get(i - 1).getFiducialId() == 7 ||  result.getTargets().get(i - 1).getFiducialId() == 8){
                        led.setLedMode(LED.Mode.VISIONON);
                        break;
                    }
                }
            Optional<EstimatedRobotPose> optEstimation = estimator.update(result);
            if (optEstimation.isEmpty())
                return Optional.empty();
            EstimatedRobotPose estimation = optEstimation.get();

            if (estimation.targetsUsed.size() == 1) {
                double ambiguity = estimation.targetsUsed.get(0).getPoseAmbiguity();
                if (ambiguity < kConfig.kPoseAmbiguityCutOff || ambiguity == -1)
                    return Optional.empty();
            }

            return Optional.ofNullable(new CameraEstimatorMeasurement(estimation, result));
        }

        /**
         * Returns whether or not to ignore a frame
         * Cases in which a frame should be ignored include when there are no targets in view,
         * if there more targets than physically possible, or an impossible combination of target are                       in view
         * @param frame the frame to be checked
         * @return whether or not to ignore a frame
         */
        private boolean ignoreFrame(PhotonPipelineResult frame) {
            if (!frame.hasTargets() || frame.getTargets().size() > kConfig.kMaxFrameFIDs)
                return true;
            else if (isDuplicate(frame))
                return true;
            else if (!isPossible(frame))
                return true;
            else
                return false;
        }
        /**
         * Returns if a frame is a duplicate of the last frame
         * Accomplishes this by comparing the timestamp of the last frame with the passed in frame
         * @param frame the frame to be checked
         * @return if the frame is a duplicate
         */
        private boolean isDuplicate(PhotonPipelineResult frame) {
            boolean duplicate = frame.getTimestampSeconds() == lastTimeStamp;
            lastTimeStamp = frame.getTimestampSeconds();
            return duplicate;
        }
        /**
         * Returns true is the combination of targets in view is physically possible
         * @param frame the frame to be checked
         * @return if the frame is possible
         */
        private boolean isPossible(PhotonPipelineResult frame) {
            boolean possible = false;
            List<Integer> ids = frame.targets.stream().map(t -> t.getFiducialId()).toList();
            for (Set<Integer> possibleFIDCombo : kConfig.kPossibleFIDCombinations) {
                possible = possibleFIDCombo.containsAll(ids);
                if (possible)
                    break;
            }
            if (!possible)
                ConsoleLogger.consoleLog("Impossible FIDs combination: " + ids);
            return possible;
        }

    }
    /**
     * A record that stores and computes deviation of a target( a tag)
     */
    public static record UnitDeviationParams(double distanceMultiplier, double eulerMultiplier, double minimum) {
        private double computeUnitDeviation(double averageDistance) {
            return Math.max(minimum, eulerMultiplier * Math.exp(averageDistance * distanceMultiplier));
        }
    }
    /**
     * A record that stores a tag's data with their deviation
     */
    public static record TagCountDeviation(UnitDeviationParams xParams, UnitDeviationParams yParams, UnitDeviationParams thetaParams) {
        private Vector<N3> computeDeviation(double averageDistance) {
            return VecBuilder.fill(
                    xParams.computeUnitDeviation(averageDistance),
                    yParams.computeUnitDeviation(averageDistance),
                    thetaParams.computeUnitDeviation(averageDistance));
        }

        public TagCountDeviation(UnitDeviationParams xyParams, UnitDeviationParams thetaParams) {
            this(xyParams, xyParams, thetaParams);
        }
    }

}
