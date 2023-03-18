package frc.team670.mustanglib.utils;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.team670.mustanglib.constants.SwerveConfig;
import frc.team670.mustanglib.swervelib.SdsModuleConfigurations;
import frc.team670.robot.constants.RobotConstants;

public class SwerveSkewTest {
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 5676.0 / 60.0
                        * SdsModuleConfigurations.MK4I_L1.getDriveReduction()
                        * SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;
        public static final double MAX_VOLTAGE = 12.0;

        public static void main(String[] args) {

                MustangSwerveDriveKinematics m_kinematics = new MustangSwerveDriveKinematics(
                                // Front left
                                new Translation2d(0.6096 / 2.0,
                                                0.6096 / 2.0),
                                // Front right
                                new Translation2d(0.6096 / 2.0,
                                                -0.6096 / 2.0),
                                // Back left
                                new Translation2d(-0.6096 / 2.0,
                                                0.6096 / 2.0),
                                // Back right
                                new Translation2d(-0.6096 / 2.0,
                                                -0.6096 / 2.0));

                ChassisSpeeds speed = new ChassisSpeeds(2, 0, 8);
                SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speed);
                // m_kinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

                System.out.println(Arrays.toString(states));

        }
}
