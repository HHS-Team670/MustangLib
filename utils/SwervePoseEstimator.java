package frc.team670.mustanglib.utils;

import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.subsystems.VisionSubsystemBase;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;

public class SwervePoseEstimator {

  private final SwerveDrive driveBase;
  private VisionSubsystemBase vision;

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
  }

  public void addTrajectory(Trajectory traj) {
    field2d.getObject("Trajectory").setTrajectory(traj);
  }

  public void removeTrajectory() {
    field2d.getObject("Trajectory").close();
  }

  public void update() {
    if (vision != null) {
      EstimatedRobotPose[] visionPoses =
          vision.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());

      for (EstimatedRobotPose pose : visionPoses) {
        if (pose != null)
          poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
      }
    }
    poseEstimator.update(driveBase.getGyroscopeRotation(), driveBase.getModulePositions());

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

}
