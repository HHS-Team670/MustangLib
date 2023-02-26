package frc.team670.mustanglib.utils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.VisionSubsystemBase;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;
import frc.team670.robot.constants.FieldConstants;

public class SwervePoseEstimator {

  private final SwerveDrive driveBase;
  private VisionSubsystemBase vision;
  private Set<Pose2d> closestTargetsSet = new HashSet<>(3);

  /**
   * Standard deviations of model states. Increase these numbers to trust your model's state
   * estimates less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians,
   * then meters.
   */
  private static final Vector<N3> stateStdDevs =
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global
   * measurements from vision less. This matrix is in the form [x, y, theta]ᵀ, with units in meters
   * and radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs =
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

  private SwerveDrivePoseEstimator poseEstimator;

  private final Field2d field2d = new Field2d();

  // private double previousPipelineTimestamp = 0;

  public SwervePoseEstimator(SwerveDrive swerve) {
    this.driveBase = swerve;
    this.vision = null;
    poseEstimator = new SwerveDrivePoseEstimator(swerve.getSwerveKinematics(),
        swerve.getGyroscopeRotation(), swerve.getModulePositions(), swerve.getOdometerPose(),
        stateStdDevs, visionMeasurementStdDevs);
    SmartDashboard.putData(field2d);
  }

  public SwervePoseEstimator(VisionSubsystemBase vision, SwerveDrive swerve) {
    this.vision = vision;
    this.driveBase = swerve;
    poseEstimator = new SwerveDrivePoseEstimator(swerve.getSwerveKinematics(),
        swerve.getGyroscopeRotation(), swerve.getModulePositions(), swerve.getOdometerPose(),
        stateStdDevs, visionMeasurementStdDevs);
    SmartDashboard.putData(field2d);
    
    List<Pose2d> allTargets = Arrays.asList(FieldConstants.Grids.scoringPoses);
    allTargets.addAll(Arrays.asList(FieldConstants.LoadingZone.loadingZoneIntakeTranslations));
    addTargetsToField(allTargets);
  }

  private void addTargetsToField(List<Pose2d> targets) {
    targets.forEach(
      (t) -> {
        addTargetToField(t);
      }
    );
  }

  private void addTargetToField(Pose2d target) {
    field2d.getObject(String.format("Target %f, %f", target.getX(), target.getY())).setPose(target);
  }

  private void removeTargetFromField(Pose2d target) {
    field2d.getObject(String.format("Target %f, %f", target.getX(), target.getY())).close();
  }

  private void updateTargets(List<Pose2d> newClosestTargets) {
    if (closestTargetsSet.isEmpty()) {
      closestTargetsSet.addAll(newClosestTargets);
      closestTargetsSet.forEach(
        (cTarget) -> {
            addTargetToField(cTarget);
        }
      );
      return;
    }

    // remove targets that are not in closest closest 3 targets
    closestTargetsSet.forEach(
      (cTarget) -> {   
        if (!newClosestTargets.contains(cTarget)) {
          closestTargetsSet.remove(cTarget);
          removeTargetFromField(cTarget);
        }
      }
    );

    // add targets that are not already in
    closestTargetsSet.addAll(newClosestTargets);
    addTargetsToField(newClosestTargets);
  }


  public void addTrajectory(Trajectory traj) {
    field2d.getObject("Trajectory").setTrajectory(traj);
  }

  public void removeTrajectory() {
    field2d.getObject("Trajectory").close();
  }

  public void update() {
    if (vision != null) {
      for (EstimatedRobotPose p : vision.getEstimatedGlobalPose(getCurrentPose())) {
        if (p != null) {
          poseEstimator.addVisionMeasurement(p.estimatedPose.toPose2d(), p.timestampSeconds);
        }
      }
    }
    poseEstimator.update(driveBase.getGyroscopeRotation(), driveBase.getModulePositions());
    // updateTargets(getSortedTargetTranslations().subList(0, 2));
    field2d.setRobotPose(getCurrentPose());
    SmartDashboard.putString("Estimated pose", getFormattedPose());
  }

  private String getFormattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f) %.2f degrees", pose.getX(), pose.getY(),
        pose.getRotation().getDegrees());
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called when the robot's
   * position on the field is known, like at the beginning of a match.
   * 
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(driveBase.getGyroscopeRotation(), driveBase.getModulePositions(),
        newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

  public List<Pose2d> getSortedTargetTranslations() {
    List<Pose2d> targets = new ArrayList<>();

    for (Translation2d p : FieldConstants.Grids.complexLowTranslations)
      targets.add(FieldConstants.allianceFlip(new Pose2d(p, new Rotation2d())));
    for (Pose2d p : FieldConstants.LoadingZone.loadingZoneIntakeTranslations)
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
