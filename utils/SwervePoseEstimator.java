package frc.team670.mustanglib.utils;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.VisionSubsystemBase;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;
import frc.team670.robot.commands.drivebase.SingleSubstationAlign;
import frc.team670.robot.constants.FieldConstants;
import frc.team670.robot.constants.RobotConstants;

public class SwervePoseEstimator {

    private final SwerveDrive driveBase;
    private VisionSubsystemBase vision;

    /**
     * Standard deviations of model states. Increase these numbers to trust your model's state
     * estimates less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians,
     * then meters.
     */
    private Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(0.1)); // default
    // VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

    /**
     * Standard deviations of the vision measurements. Increase these numbers to trust global
     * measurements from vision less. This matrix is in the form [x, y, theta]ᵀ, with units in
     * meters and radians.
     */
    private Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.9, 0.9, 99999);
    // VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

    private SwerveDrivePoseEstimator poseEstimator;

    private final Field2d field2d = new Field2d();

    public SwervePoseEstimator(SwerveDrive swerve) {
        this.driveBase = swerve;
        this.vision = null;
        poseEstimator = new SwerveDrivePoseEstimator(swerve.getSwerveKinematics(),
                swerve.getGyroscopeRotation(), swerve.getModulePositions(), new Pose2d(),
                stateStdDevs, visionMeasurementStdDevs);
        SmartDashboard.putData(field2d);
    }

    public void initialize(VisionSubsystemBase vision) {
        this.vision = vision;

        SmartDashboard.putNumber("vision std x: ", 0.9);
        SmartDashboard.putNumber("vision std y: ", 0.9);
        SmartDashboard.putNumber("vision std deg: ", 0.9);

        // load single sub to smartdashboard
        Pose2d singlesub = FieldConstants.allianceFlip(FieldConstants.LoadingZone.IntakePoses[0]);
        field2d.getObject("Single Substation").setPose(singlesub);
        SmartDashboard.putData(field2d);
        SmartDashboard.putString("Single Substation", getFormattedPose(singlesub));
    }


    public void addTrajectory(Trajectory traj) {
        field2d.getObject("Trajectory").setTrajectory(getAbsoluteFieldOrientedTrajectory(traj));
    }

    public void removeTrajectory() {
        field2d.getObject("Trajectory").close();
    }

    public void update() {
        if (vision != null && !DriverStation.isAutonomous()) {
            // scale vision "trust" exponentially by distance

            // find average distance
            double avgDistance = 0;
            int tagsSeen = 0;
            for (int i = 0; i < vision.getCameras().length; i++) {
                var cam = vision.getCameras()[i];
                var result = cam.getLatestResult();
                if (result.hasTargets()) {
                    for (int j = 0; j < result.targets.size(); j++) {
                        tagsSeen++;
                        var t = result.targets.get(j);
                        avgDistance += t.getBestCameraToTarget().getTranslation()
                                .getDistance(RobotConstants.CAMERA_OFFSETS[i].getTranslation());
                    }

                }
            }
            avgDistance /= tagsSeen;
            SmartDashboard.putNumber("Average distance", avgDistance);

            // scale vision xy std dev by distance
            double visionXYStdDev = 0.01 * Math.pow(avgDistance, 2) / tagsSeen;

            for (EstimatedRobotPose p : vision.getEstimatedGlobalPose(getCurrentPose())) {
                if (p != null) {
                    // poseEstimator.addVisionMeasurement(p.estimatedPose.toPose2d(),
                    //         p.timestampSeconds);
                    
                    poseEstimator.addVisionMeasurement(p.estimatedPose.toPose2d(),
                            p.timestampSeconds, VecBuilder.fill(visionXYStdDev, visionXYStdDev, 99999));

                    field2d.getObject("camera pose").setPose(FieldConstants.allianceFlip(p.estimatedPose.toPose2d()));
                }
            }
        }
        poseEstimator.update(driveBase.getGyroscopeRotation(), driveBase.getModulePositions());
        field2d.setRobotPose(getAbsoluteFieldOrientedPoseFromAllianceOriented());
        SmartDashboard.putString("Estimated Pose", getFormattedPose(getAbsoluteFieldOrientedPoseFromAllianceOriented()));
    }

    private String getFormattedPose() {
        return getFormattedPose(getCurrentPose());
    }

    private String getFormattedPose(Pose2d pose) {
        return String.format("(%.2f, %.2f) %.2f degrees", pose.getX(), pose.getY(),
                pose.getRotation().getDegrees());
    }

    /**
     * 
     * @return alliance relative pose using odometry and vision
     */
    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }

    

    private Pose2d getAbsoluteFieldOrientedPoseFromAllianceOriented() {
        return getAbsoluteFieldOrientedPoseFromAllianceOriented(getCurrentPose());
    }

    private Pose2d getAbsoluteFieldOrientedPoseFromAllianceOriented(Pose2d pose) {
        if (DriverStation.getAlliance() == Alliance.Red) {
            return FieldConstants.allianceOrientedAllianceFlip(pose);
        } else {
            return pose;
        }
    }

    private Trajectory getAbsoluteFieldOrientedTrajectory(Trajectory traj) {
        List<State> states = traj.getStates();
        List<State> adjusted = new ArrayList<>(states.size());
        states.forEach(s -> {
            if (DriverStation.getAlliance() == Alliance.Red) {
                adjusted.add(new State(s.timeSeconds, s.velocityMetersPerSecond,
                        s.accelerationMetersPerSecondSq, getAbsoluteFieldOrientedPoseFromAllianceOriented(s.poseMeters),
                        -s.curvatureRadPerMeter));
            }
        });

        return adjusted.isEmpty() ? traj : new Trajectory(adjusted);
    }

    /**
     * Resets the current pose to the specified pose. This should ONLY be called when the robot's
     * position on the field is known, like at the beginning of a match.
     * 
     * @param newPose new pose
     */
    public void setCurrentPose(Pose2d newPose) {
        poseEstimator.resetPosition(driveBase.getGyroscopeRotation(),
                driveBase.getModulePositions(), newPose);
    }

    /**
     * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
     * what "forward" is for field oriented driving.
     */
    public void resetFieldPosition() {
        setCurrentPose(new Pose2d());
    }

    public VisionSubsystemBase getVision() {
        return vision;
    }



    public List<Pose2d> getSortedTargetTranslations() {
        List<Pose2d> targets = new ArrayList<>();

        for (Translation2d p : FieldConstants.Grids.complexLowTranslations)
            targets.add(FieldConstants.allianceFlip(new Pose2d(p, new Rotation2d())));
        for (Pose2d p : FieldConstants.LoadingZone.IntakePoses)
            targets.add(FieldConstants.allianceFlip(p));

        Translation2d robotTranslation = getCurrentPose().getTranslation();
        targets.sort(new Comparator<Pose2d>() {
            @Override
            public int compare(Pose2d p1, Pose2d p2) {
                if (p1.getTranslation().getDistance(robotTranslation) > p2.getTranslation()
                        .getDistance(robotTranslation))
                    return 1;
                else if (p1.getTranslation().getDistance(robotTranslation) < p2.getTranslation()
                        .getDistance(robotTranslation))
                    return -1;
                else
                    return 0;
            }
        });
        return targets;
    }

}
