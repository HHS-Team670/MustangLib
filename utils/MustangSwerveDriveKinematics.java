package frc.team670.mustanglib.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.constants.RobotConstantsBase;
import frc.team670.mustanglib.swervelib.SdsModuleConfigurations;

public class MustangSwerveDriveKinematics extends SwerveDriveKinematics {

    public MustangSwerveDriveKinematics(Translation2d... wheelsMeters) {
        super(wheelsMeters);
    }

    @Override
    public SwerveModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds) {

        SmartDashboard.putBoolean("mustangsdk called", true);

        Pose2d robot_pose_vel = new Pose2d(chassisSpeeds.vxMetersPerSecond * RobotConstantsBase.LOOP_TIME,
                chassisSpeeds.vyMetersPerSecond * RobotConstantsBase.LOOP_TIME,
                Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond * RobotConstantsBase.LOOP_TIME));
        Twist2d t2d = (new Pose2d()).log(robot_pose_vel);
        //Twist2d t2d = Pose2d.log(robot_pose_vel);
        System.out.println("normal: " + chassisSpeeds);
        ChassisSpeeds updatedChassisSpeeds = new ChassisSpeeds(t2d.dx / RobotConstantsBase.LOOP_TIME,
                t2d.dy / RobotConstantsBase.LOOP_TIME, t2d.dtheta / RobotConstantsBase.LOOP_TIME);
        System.out.println("updated: " + updatedChassisSpeeds);
        SwerveModuleState[] states = super.toSwerveModuleStates(updatedChassisSpeeds);
        desaturateWheelSpeeds(states, 5676.0 / 60.0 
            * SdsModuleConfigurations.MK4I_L1.getDriveReduction() 
            * SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI);
        return states;
    }

//              Pose2d robot_pose_vel = new Pose2d(mPeriodicIO.des_chassis_speeds.vxMetersPerSecond * Constants.kLooperDt,
//              mPeriodicIO.des_chassis_speeds.vyMetersPerSecond * Constants.kLooperDt,
//              Rotation2d.fromRadians(mPeriodicIO.des_chassis_speeds.omegaRadiansPerSecond * Constants.kLooperDt));

//              Twist2d twist_vel = Pose2d.log(robot_pose_vel);

//              ChassisSpeeds updated_chassis_speeds = new ChassisSpeeds(
//              twist_vel.dx / Constants.kLooperDt, twist_vel.dy / Constants.kLooperDt, twist_vel.dtheta / Constants.kLooperDt);`

}