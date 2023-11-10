package frc.team670.mustanglib.utils;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import frc.team670.mustanglib.subsystems.VisionSubsystemBase;
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

// import frc.team670.mustanglib.subsystems.VisionfSubsystemBase;
import frc.team670.mustanglib.subsystems.VisionSubsystemBase.VisionMeasurement;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;


/**
 * Wrapper class for SwerveDrivePoseEstimator. Incorporates Swerve Drive and Vision 
 * 
 * @author ethan c
 */
public abstract class SwervePoseEstimatorBase {

    private final SwerveDrive driveBase;
    private VisionSubsystemBase vision;
    private final String DRIVEBASE_ESTIMATED_POSE;
    /**
     * Standard deviations of model states. Increase these numbers to trust your
     * model's state
     * estimates less. This matrix is in the form [x, y, theta]ᵀ, with units in
     * meters and radians,
     * then meters.
     */
    private Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(0.1)); // default
    // VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

    /**
     * Standard deviations of the vision measurements. Increase these numbers to
     * trust global
     * measurements from vision less. This matrix is in the form [x, y, theta]ᵀ,
     * with units in
     * meters and radians.
     */
    private Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.9, 0.9, 99999);
    // VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

    private SwerveDrivePoseEstimator poseEstimator;

    private final Field2d field2d = new Field2d();

    public SwervePoseEstimatorBase(SwerveDrive swerve) {
        DRIVEBASE_ESTIMATED_POSE = "DriveBase/Estimated Pose";
        this.driveBase = swerve;
        this.vision = null;
        poseEstimator = new SwerveDrivePoseEstimator(swerve.getSwerveKinematics(),
                swerve.getGyroscopeRotation(), swerve.getModulePositions(), new Pose2d(),
                stateStdDevs, visionMeasurementStdDevs);
        SmartDashboard.putData(field2d);
    }

    public void initVision(VisionSubsystemBase vision) {
        this.vision = vision;

        SmartDashboard.putNumber("vision std x: ", 0.9);
        SmartDashboard.putNumber("vision std y: ", 0.9);
        SmartDashboard.putNumber("vision std deg: ", 0.9);

       // The commented code is loading a single substation pose to the SmartDashboard. It flips the
       // pose based on the alliance color and sets it as the pose for the "Single Substation" object
       // in the Field2d. It then puts the Field2d object and the formatted pose on the SmartDashboard.
       //TODO move to 2023 // // load single sub to smartdashboard 
        // Pose2d singlesub = FieldConstants.allianceFlip(FieldConstants.LoadingZone.IntakePoses[0]);
        // field2d.getObject("Single Substation").setPose(singlesub);
        // SmartDashboard.putData(field2d);
        // SmartDashboard.putString("Single Substation", getFormattedPose(singlesub));
    }

    public void addTrajectory(Trajectory traj) {
        field2d.getObject("Trajectory").setTrajectory(getAbsoluteFieldOrientedTrajectory(traj));
    }

    public void removeTrajectory() {
        field2d.getObject("Trajectory").close();
    }
    /**
     * Updates the robot pose based on vision estimates and swerve encoders
     */
    public void update() {
        // if (vision != null && !DriverStation.isAutonomous()) {
        if (vision != null){

            while (!vision.isMeasurementBufferEmpty()) {
                VisionMeasurement m = vision.getVisionMeasurement();
                EstimatedRobotPose estimation = m.estimation();
                Pose2d estimatedPose = estimation.estimatedPose.toPose2d();

                poseEstimator.addVisionMeasurement(estimatedPose, estimation.timestampSeconds, m.confidence());

                field2d.getObject("camera pose").setPose(estimatedPose);
            }
        }

        poseEstimator.update(driveBase.getGyroscopeRotation(), driveBase.getModulePositions());
        field2d.setRobotPose(getAbsoluteFieldOrientedPoseFromAllianceOriented());
        Logger.getInstance().recordOutput(DRIVEBASE_ESTIMATED_POSE,
                getAbsoluteFieldOrientedPoseFromAllianceOriented());
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
    /**
     * 
     * @return the robot position on the field relative to the current alliance side
     * @see #getAbsoluteFieldOrientedPoseFromAllianceOriented(Pose2d pose) 
     */
    private Pose2d getAbsoluteFieldOrientedPoseFromAllianceOriented() {
        return getAbsoluteFieldOrientedPoseFromAllianceOriented(getCurrentPose());
    }

    /**
     * The function returns the absolute field-oriented pose if the alliance is not red, otherwise it
     * returns the pose with the alliance flip applied.
     * 
     * @param pose The "pose" parameter is a Pose2d object representing the robot's position and
     * orientation in the field.
     * @return The method is returning a Pose2d object.
     */
    protected abstract Pose2d getAbsoluteFieldOrientedPoseFromAllianceOriented(Pose2d pose);
    /**
     * Transforms the given trajectory into one relative to the field (based on alliance color)
     * @param traj the trajectory to be tranformed
     * @return The transformed trajectory (if the Alliance is red then then a deep copy will be transformed and made else the traj object will be returned)
     */
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
     * Resets the current pose to the specified pose. This should ONLY be called
     * when the robot's
     * position on the field is known, like at the beginning of a match.
     * 
     * @param newPose new pose
     */
    public void setCurrentPose(Pose2d newPose) {
        poseEstimator.resetPosition(driveBase.getGyroscopeRotation(),
                driveBase.getModulePositions(), newPose);
    }

    /**
     * Resets the position on the field to 0,0 0-degrees, with forward being
     * downfield. This resets
     * what "forward" is for field oriented driving.
     */
    public void resetFieldPosition() {
        setCurrentPose(new Pose2d());
    }

    /**
     * The function returns an instance of the VisionSubsystemBase class.
     * 
     * @return Return the VisionSubystemBase currently used
     */
    public VisionSubsystemBase getVision() {
        return vision;
    }
    //Returns scoring and intaking targets
    protected abstract List<Pose2d> getTargets();

    /**
     * The function returns a list of target (AprilTag) translations sorted based on their distance from the
     * current robot position. (Ascending)
     * 
     * @return The method is returning a List of Pose2d objects sorted by distance from the robot
     */
    public List<Pose2d> getSortedTargetTranslations() {
        List<Pose2d> targets = getTargets();

        // for (Translation2d p : FieldConstants.Grids.complexLowTranslations)
        //     targets.add(FieldConstants.allianceFlip(new Pose2d(p, new Rotation2d())));
        // for (Pose2d p : FieldConstants.LoadingZone.IntakePoses)
        //     targets.add(FieldConstants.allianceFlip(p));

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
