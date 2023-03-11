package frc.team670.mustanglib.utils;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.team670.robot.constants.FieldConstants;

public class SwervePoseEstimator {

    private final SwerveDrive driveBase;
    private VisionSubsystemBase vision;
    // private Set<Pose2d> closestTargetsSet = new HashSet<>(3);

    /**
     * Standard deviations of model states. Increase these numbers to trust your model's state
     * estimates less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians,
     * then meters.
     */
    private static final Vector<N3> stateStdDevs =
            VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(0.1)); // default
            // VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

    /**
     * Standard deviations of the vision measurements. Increase these numbers to trust global
     * measurements from vision less. This matrix is in the form [x, y, theta]ᵀ, with units in
     * meters and radians.
     */
    private static final Vector<N3> visionMeasurementStdDevs =
            VecBuilder.fill(0.9, 0.9, Units.degreesToRadians(0.9));  // default
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

        SmartDashboard.putData(field2d);
        List<Pose2d> allTargets = new ArrayList<>();
        for (Pose2d p : FieldConstants.Grids.scoringPoses) {
            p = FieldConstants.allianceFlip(p);
            allTargets.add(p);
        }
        for (Pose2d p : FieldConstants.LoadingZone.IntakePoses) {
            p = FieldConstants.allianceFlip(p);
            allTargets.add(p);
        }
        // addTargetsToField(allTargets);
    }

    // private void addTargetsToField(List<Pose2d> targets) {
    // targets.forEach((t) -> {
    // addTargetToField(t);
    // });
    // }

    // private void addTargetToField(Pose2d target) {
    // field2d.getObject(String.format("Target %f, %f", target.getX(),
    // target.getY())).setPose(target);
    // }

    public void addTrajectory(Trajectory traj) {
        field2d.getObject("Trajectory").setTrajectory(getAbsoluteFieldOrientedTrajectory(traj));
    }

    public void removeTrajectory() {
        field2d.getObject("Trajectory").close();
    }

    public void update() {
        if (vision != null) {
            SmartDashboard.putBoolean("VISION IS: ", vision != null);
            for (EstimatedRobotPose p : vision.getEstimatedGlobalPose(getCurrentPose())) {
                if (p != null && !DriverStation.isAutonomous()) {
                    poseEstimator.addVisionMeasurement(p.estimatedPose.toPose2d(),   // TODO: testing auton without vision
                            p.timestampSeconds);
                }
            }
        }
        poseEstimator.update(driveBase.getGyroscopeRotation(), driveBase.getModulePositions());
        // updateTargets(getSortedTargetTranslations().subList(0, 2));
        field2d.setRobotPose(getAbsoluteFieldOrientedPose());
        SmartDashboard.putString("Estimated pose", getFormattedPose());
    }

    private String getFormattedPose() {
        var pose = getCurrentPose();
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

    public Pose2d getAbsoluteFieldOrientedPose() {
        return getAbsoluteFieldOrientedPose(getCurrentPose());
    }

    private Pose2d getAbsoluteFieldOrientedPose(Pose2d pose) {
        if (DriverStation.getAlliance() == Alliance.Red) {
            return FieldConstants.allianceOrientedAllianceFlip(pose);
        } else {
            return pose;
        }
    }
    
    private Trajectory getAbsoluteFieldOrientedTrajectory(Trajectory traj) {
        List<State> states = traj.getStates();
        List<State> adjusted = new ArrayList<>(states.size());
        states.forEach(
            s -> {
                if (DriverStation.getAlliance() == Alliance.Red) {
                    adjusted.add(new State(s.timeSeconds, s.velocityMetersPerSecond, s.accelerationMetersPerSecondSq, getAbsoluteFieldOrientedPose(s.poseMeters), -s.curvatureRadPerMeter));
                }
            }
        );

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
