package frc.team670.mustanglib.subsystems;
// // package frc.team670.mustanglib.subsystems;
// //Why is this commented? Idk i just wanted to see if it works
// // import java.util.List;
// // import java.util.Map;
// // import java.util.Optional;
// // import java.util.Set;
// // import java.util.concurrent.ConcurrentLinkedQueue;
// // import java.util.concurrent.Executors;
// // import java.util.concurrent.ThreadPoolExecutor;

// // import org.photonvision.EstimatedRobotPose;
// // import org.photonvision.PhotonCamera;
// // import org.photonvision.PhotonPoseEstimator;
// // import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// // import org.photonvision.targeting.PhotonPipelineResult;

// // import edu.wpi.first.apriltag.AprilTagFieldLayout;
// // import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
// // import edu.wpi.first.math.MathUtil;
// // import edu.wpi.first.math.VecBuilder;
// // import edu.wpi.first.math.Vector;
// // import edu.wpi.first.math.geometry.Transform3d;
// // import edu.wpi.first.math.numbers.N3;
// // import edu.wpi.first.wpilibj.DriverStation;
// // import edu.wpi.first.wpilibj.DriverStation.Alliance;
// // import frc.team670.mustanglib.utils.Logger;

// // /**
// //  * Subsystem base vision. Mainly used for april tags pose estimation.
// //  * 
// //  * @author ethan c
// //  */
// // public abstract class VisionSubsystemBase extends MustangSubsystemBase {
// //     private Config kConfig;
// //     private AprilTagFieldLayout kFieldLayout;

// //     protected PhotonCamera[] mCameras;
// //     protected CameraPoseEstimator[] mCameraEstimators;
// //     private ThreadPoolExecutor mExecutor = (ThreadPoolExecutor) Executors.newFixedThreadPool(5);
// //     private ConcurrentLinkedQueue<VisionMeasurement> mVisionMeasurementsBuffer;
// //     private boolean mInit = false;

// //     public static record Config(AprilTagFieldLayout kFieldLayout, String[] kCameraIDs,
// //             Transform3d[] kCameraOffsets, double kPoseAmbiguityCutOff, int kMaxFrameFIDs,
// //             List<Set<Integer>> kPossibleFIDCombinations, Map<Integer, TagCountDeviation> kVisionStdFromTagsSeen) {
// //     }

// //     public VisionSubsystemBase(Config config) {
// //         this.kConfig = config;
// //         this.kFieldLayout = config.kFieldLayout;
// //         this.mVisionMeasurementsBuffer = new ConcurrentLinkedQueue<>();
// //         this.mCameras = new PhotonCamera[config.kCameraIDs.length];
// //         for (int i = 0; i < mCameras.length; i++) {
// //             mCameras[i] = new PhotonCamera(config.kCameraIDs[i]);
// //         }
// //     }

// //     /**
// //      * DO NOT CALL IN ROBOT INIT! DS IS NOT NECESSARILY READY THEN. CALL IN PERIODIC
// //      * OR AUTONINIT.
// //      * More details here:
// //      * https://www.chiefdelphi.com/t/getalliance-always-returning-red/425782/27
// //      */
// //     public void initalize() {
// //         // does nothing if DS not initialized yet
// //         if (DriverStation.getAlliance() == Alliance.Invalid) {
// //             mInit = false;
// //             return;
// //         }
// //         setFieldOrigin();

// //         CameraPoseEstimator[] c = new CameraPoseEstimator[mCameras.length];
// //         for (int i = 0; i < c.length; i++) {
// //             c[i] = new CameraPoseEstimator(mCameras[i], kConfig.kCameraOffsets[i], kFieldLayout);
// //         }
// //         this.mCameraEstimators = c;
// //         mInit = true;
// //     }

// //     private void setFieldOrigin() {
// //         var origin = DriverStation.getAlliance() == Alliance.Blue
// //                 ? OriginPosition.kBlueAllianceWallRightSide
// //                 : OriginPosition.kRedAllianceWallRightSide;
// //         kFieldLayout.setOrigin(origin);
// //     }

// //     @Override
// //     public void mustangPeriodic() {
// //         // attempt initialization until initialized
// //         if (!mInit) {
// //             initalize();
// //             return;
// //         }

// //         for (CameraPoseEstimator c : mCameraEstimators) {
// //             mExecutor.execute(
// //                     () -> {
// //                         processVisionFeed(c);
// //                     });
// //         }
// //     }

// //     public boolean isInitialized() {
// //         return mInit;
// //     }

// //     public record VisionMeasurement(EstimatedRobotPose estimation, Vector<N3> confidence) {
// //     }

// //     public VisionMeasurement getVisionMeasurement() {
// //         return mVisionMeasurementsBuffer.poll();
// //     }

//     /** @return the healthstate of the vision subsystem */
//     @Override
//     public HealthState checkHealth() {
//         HealthState state = HealthState.GREEN;
//         int counter = 0;
//         // checks through the photon cameras and checks if they are null or !connected
//         for (PhotonCamera camera: mCameras) {
//             if(camera == null || !camera.isConnected()){
//                 state = HealthState.YELLOW;
//                 counter++;
//             }
//         }
//         //iff all of the cameras are null or not connected healthstate = red
//         if (counter == mCameras.length && mCameras.length!=0){
//             state = HealthState.RED;
//         }
//         return state;
//     }
    
//    /**
//     * 
//     * @return the cameras this subsystem uses
//     */
//     public PhotonCamera[] getCameras() {
//         return mCameras;
//     }

// //     private void processVisionFeed(CameraPoseEstimator cameraEstimator) {
// //         Optional<CameraPoseEstimator.CameraEstimatorMeasurement> optMeasurement = cameraEstimator.update();
// //         if (optMeasurement.isEmpty())
// //             return;
// //         CameraPoseEstimator.CameraEstimatorMeasurement measurement = optMeasurement.get();
// //         EstimatedRobotPose estimation = measurement.estimation;
// //         double avgDistance = getAverageDistance(estimation); // TODO: figure out how vision std work
// //         Vector<N3> deviation = kConfig.kVisionStdFromTagsSeen.get(MathUtil.clamp(estimation.targetsUsed.size(),
// //                 0,
// //                 kConfig.kVisionStdFromTagsSeen.keySet().size()))
// //                 .computeDeviation(avgDistance);
// //         mVisionMeasurementsBuffer.add(new VisionMeasurement(estimation, deviation));

// //     }

// //     private double getAverageDistance(EstimatedRobotPose estimation) {
// //         double sumDistance = 0;
// //         for (var target : estimation.targetsUsed) {
// //             var t3d = target.getBestCameraToTarget();
// //             sumDistance += Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
// //         }
// //         return sumDistance / estimation.targetsUsed.size();
// //     }

// //     @Override
// //     public HealthState checkHealth() {
// //         return HealthState.GREEN;
// //     }

// //     public PhotonCamera[] getCameras() {
// //         return mCameras;
// //     }

// //     private class CameraPoseEstimator {
// //         private PhotonCamera photonCamera;
// //         private PhotonPoseEstimator estimator;
// //         private double lastTimeStamp;

// //         public CameraPoseEstimator(PhotonCamera photonCamera, Transform3d robotToCam,
// //                 AprilTagFieldLayout fieldLayout) {
// //             this.photonCamera = photonCamera;
// //             estimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP,
// //                     photonCamera, robotToCam);
// //             estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
// //         }

// //         public static record CameraEstimatorMeasurement(EstimatedRobotPose estimation,
// //                 PhotonPipelineResult result) {
// //         }

// //         /**
// //          * Updates the Camera estimator. Should be called periodically
// //          * 
// //          * @return An Optional of the Photon pose estimator estimation and pipline
// //          *         result.
// //          *         Bad frames are ignored.
// //          */
// //         public Optional<CameraEstimatorMeasurement> update() {
// //             PhotonPipelineResult result = photonCamera.getLatestResult();
// //             if (ignoreFrame(result))
// //                 return Optional.empty();

// //             Optional<EstimatedRobotPose> optEstimation = estimator.update(result);
// //             if (optEstimation.isEmpty())
// //                 return Optional.empty();
// //             EstimatedRobotPose estimation = optEstimation.get();

// //             if (estimation.targetsUsed.size() == 1) {
// //                 double ambiguity = estimation.targetsUsed.get(0).getPoseAmbiguity();
// //                 if (ambiguity < kConfig.kPoseAmbiguityCutOff || ambiguity == -1)
// //                     return Optional.empty();
// //             }

// //             return Optional.ofNullable(new CameraEstimatorMeasurement(estimation, result));
// //         }

// //         private boolean ignoreFrame(PhotonPipelineResult frame) {
// //             if (!frame.hasTargets() || frame.getTargets().size() > kConfig.kMaxFrameFIDs)
// //                 return true;
// //             else if (isDuplicate(frame))
// //                 return true;
// //             else if (!isPossible(frame))
// //                 return true;
// //             else
// //                 return false;
// //         }

// //         private boolean isDuplicate(PhotonPipelineResult frame) {
// //             boolean duplicate = frame.getTimestampSeconds() == lastTimeStamp;
// //             lastTimeStamp = frame.getTimestampSeconds();
// //             return duplicate;
// //         }

// //         private boolean isPossible(PhotonPipelineResult frame) {
// //             boolean possible = false;
// //             List<Integer> ids = frame.targets.stream().map(t -> t.getFiducialId()).toList();
// //             for (Set<Integer> possibleFIDCombo : kConfig.kPossibleFIDCombinations) {
// //                 possible = possibleFIDCombo.containsAll(ids);
// //                 if (possible)
// //                     break;
// //             }
// //             if (!possible)
// //                 Logger.consoleLog("Impossible FIDs combination: " + ids);
// //             return possible;
// //         }

// //     }

// //     public static record UnitDeviationParams(
// //             double distanceMultiplier, double eulerMultiplier, double minimum) {

// //         private double computeUnitDeviation(double averageDistance) {
// //             return Math.max(minimum, eulerMultiplier * Math.exp(averageDistance * distanceMultiplier));
// //         }
// //     }

// //     public static record TagCountDeviation(UnitDeviationParams xParams, UnitDeviationParams yParams,
// //             UnitDeviationParams thetaParams) {
// //         private Vector<N3> computeDeviation(double averageDistance) {
// //             return VecBuilder.fill(
// //                     xParams.computeUnitDeviation(averageDistance),
// //                     yParams.computeUnitDeviation(averageDistance),
// //                     thetaParams.computeUnitDeviation(averageDistance));
// //         }

// //         public TagCountDeviation(UnitDeviationParams xyParams, UnitDeviationParams thetaParams) {
// //             this(xyParams, xyParams, thetaParams);
// //         }
// //     }

// // }
