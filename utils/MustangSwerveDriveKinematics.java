package frc.team670.mustanglib.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.constraint.RectangularRegionConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.constants.RobotConstantsBase;
import frc.team670.mustanglib.swervelib.SdsModuleConfigurations;
import frc.team670.mustanglib.utils.math.MustangPose2d;
import frc.team670.mustanglib.utils.math.Rotation;

public class MustangSwerveDriveKinematics extends SwerveDriveKinematics {

    public MustangSwerveDriveKinematics(Translation2d... wheelsMeters) {
        super(wheelsMeters);
    }

    // @Override
    // public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds) {

    //     SmartDashboard.putBoolean("mustangsdk called", true);

    //     Pose2d robot_pose_vel = new Pose2d(chassisSpeeds.vxMetersPerSecond * RobotConstantsBase.LOOP_TIME,
    //             chassisSpeeds.vyMetersPerSecond * RobotConstantsBase.LOOP_TIME,
    //             Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond * RobotConstantsBase.LOOP_TIME));
    //     Twist2d t2d = (new MustangPose2d()).log(robot_pose_vel);
    //     //Twist2d t2d = Pose2d.log(robot_pose_vel);
    //     System.out.println("normal: " + chassisSpeeds);
    //     ChassisSpeeds updatedChassisSpeeds = new ChassisSpeeds(t2d.dx / RobotConstantsBase.LOOP_TIME,
    //             t2d.dy / RobotConstantsBase.LOOP_TIME, t2d.dtheta / RobotConstantsBase.LOOP_TIME);
    //     System.out.println("updated: " + updatedChassisSpeeds);
    //     SwerveModuleState[] states = super.toSwerveModuleStates(updatedChassisSpeeds);
    //     desaturateWheelSpeeds(states, 5676.0 / 60.0 
    //         * SdsModuleConfigurations.MK4I_L1.getDriveReduction() 
    //         * SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI);
    //     return states;
    // }

    @Override
    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
        Translation2d robotPoseVel = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
        if (chassisSpeeds.omegaRadiansPerSecond < 0 ) {
            robotPoseVel = robotPoseVel.rotateBy(Rotation2d.fromRadians(-Math.PI/2));
        } else if (chassisSpeeds.omegaRadiansPerSecond > 0 ) {
            robotPoseVel = robotPoseVel.rotateBy(Rotation2d.fromRadians(Math.PI/2));
        }
        
        Transform2d offset = new Transform2d(robotPoseVel.times(0.3), new Rotation2d());
        Pose2d updatedSpeeds = new Pose2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond)).transformBy(offset);
        System.out.println("normal: " + chassisSpeeds);
        ChassisSpeeds updatedChassisSpeeds = new ChassisSpeeds(updatedSpeeds.getX(),
                    updatedSpeeds.getY(), updatedSpeeds.getRotation().getRadians()); 
        SwerveModuleState[] states = super.toSwerveModuleStates(updatedChassisSpeeds);
        System.out.println("updated: " + updatedChassisSpeeds);
        desaturateWheelSpeeds(states, 5676.0 / 60.0 
            * SdsModuleConfigurations.MK4I_L1.getDriveReduction() 
            * SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI);
        return states;
    }

}