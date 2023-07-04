package frc.team670.mustanglib;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;
import java.util.Map;
import static java.util.Map.entry;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import frc.team670.mustanglib.subsystems.drivebase.SwerveDrive;
import frc.team670.mustanglib.swervelib.Mk4iSwerveModuleHelper.GearRatio;
import frc.team670.mustanglib.swervelib.ModuleConfiguration;
import frc.team670.mustanglib.swervelib.SdsModuleConfigurations;
/**
 * A place for storing constants required in mustanglib. 
 * This includes most drivebase constants. For every robot, add mac address like below as well as relevant constants
 * 
 * Update every year
 */
public class RobotConstantsBase {

    public static final String kSunTzuAddress = "00:80:2F:34:0B:07";
    public static final String kSkipperAddress = "00:80:2F:33:D0:46";
    public static final String kRobotAddress = getMACAddress();
    
    private static  Map<String, Double> robotSpecificConstants = Map.ofEntries(
        entry(kSunTzuAddress, Map.ofEntries(
                    entry("kBackRightModuleSteerOffsetRadians", -Math.toRadians(90.895)),
                    entry("kBackLeftModuleSteerOffsetRadians", -Math.toRadians(299.807)),
                    entry("kFrontRightModuleSteerOffsetRadians", -Math.toRadians(137.499)),
                    entry("kFrontLeftModuleSteerOffsetRadians", -Math.toRadians(318.604)),
                    entry("kSwerveModuleConfig", 2.0))),
            entry(kSkipperAddress,
                    Map.ofEntries(
                            entry("kBackRightModuleSteerOffsetRadians", -Math.toRadians(82.694)),
                            entry("kBackLeftModuleSteerOffsetRadians", -Math.toRadians(233.29)),
                            entry("kFrontRightModuleSteerOffsetRadians", -Math.toRadians(225.77)),
                            entry("kFrontLeftModuleSteerOffsetRadians", -Math.toRadians(112.53)),
                            entry("kSwerveModuleConfig", 1.0))))
            .get(kRobotAddress);
            
    public static final class SwerveDriveBase {
        
        public static final double kWidth = Units.inchesToMeters(36);
        public static double kClearance = Math.hypot(kWidth, kWidth) / 2 + 0.05;
        public static final double kTrackWidthMeters = 0.6096;
        public static final double kWheelBaseMeters = 0.6096;

        public static final ModuleConfiguration kModuleConfig = robotSpecificConstants.get("kSwerveModuleConfig") == 1.0
                ? SdsModuleConfigurations.MK4I_L1
                : SdsModuleConfigurations.MK4I_L2;

        public static final GearRatio kSwerveModuleGearRatio = robotSpecificConstants.get("kSwerveModuleConfig") == 1.0
                ? GearRatio.L1
                : GearRatio.L2;

        public static final int kFrontLeftModuleSteerMotorID = 20;
        public static final int kFrontLeftModuleDriveMotorID = 21;
        public static final int kFrontLeftModuleSteerEncoderID = 30;
        public static final double kFrontLeftModuleSteerOffsetRadians = robotSpecificConstants
                .get("kFrontLeftModuleSteerOffsetRadians");

        public static final int kFrontRightModuleSteerMotorID = 22;
        public static final int kFrontRightModuleDriveMotorID = 23;
        public static final int kFrontRightModuleSteerEncoderID = 32;
        public static final double kFrontRightModuleSteerOffsetRadians = robotSpecificConstants
                .get("kFrontRightModuleSteerOffsetRadians");

        public static final int kBackLeftModuleSteerMotorID = 26;
        public static final int kBackLeftModuleDriveMotorID = 27;
        public static final int kBackLeftModuleSteerEncoderID = 36;
        public static final double kBackLeftModuleSteerOffsetRadians = robotSpecificConstants
                .get("kBackLeftModuleSteerOffsetRadians");

        public static final int kBackRightModuleSteerMotorID = 24;
        public static final int kBackRightModuleDriveMotorID = 25;
        public static final int kBackRightModuleSteerEncoderID = 34;
        public static final double kBackRightModuleSteerOffsetRadians = robotSpecificConstants
                .get("kBackRightModuleSteerOffsetRadians");

        public final static SerialPort.Port kNAVXPort = SerialPort.Port.kMXP;
        public static final double kPitchOffset = 2;

        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 4;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI * 8;

        public static final double kMaxVoltage = 12.0;
        public static final double kMaxDriveCurrent = 45.0;
        public static final double kMaxSteerCurrent = 20.0;

        // The formula for calculating the theoretical maximum velocity is:
        // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
        // pi
        // An example of this constant for a Mk4 L2 module with NEOs to drive is:
        // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
        // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
        public static final double kMaxVelocityMetersPerSecond = 5676.0 / 60.0
                * kModuleConfig.getDriveReduction() * kModuleConfig.getWheelDiameter() * Math.PI;

        public static final double kMaxAngularVelocityRadiansPerSecond = kMaxVelocityMetersPerSecond
                / Math.hypot(kTrackWidthMeters / 2.0, kWheelBaseMeters / 2.0);

        public static final SwerveDrive.Config kConfig = new SwerveDrive.Config(kTrackWidthMeters,
                kWheelBaseMeters, kMaxVelocityMetersPerSecond, kMaxVoltage, kMaxDriveCurrent,
                kMaxSteerCurrent, kNAVXPort, kSwerveModuleGearRatio, kFrontLeftModuleDriveMotorID,
                kFrontLeftModuleSteerMotorID, kFrontLeftModuleSteerEncoderID,
                kFrontLeftModuleSteerOffsetRadians, kFrontRightModuleDriveMotorID,
                kFrontRightModuleSteerMotorID, kFrontRightModuleSteerEncoderID,
                kFrontRightModuleSteerOffsetRadians, kBackLeftModuleDriveMotorID,
                kBackLeftModuleSteerMotorID, kBackLeftModuleSteerEncoderID,
                kBackLeftModuleSteerOffsetRadians, kBackRightModuleDriveMotorID,
                kBackRightModuleSteerMotorID, kBackRightModuleSteerEncoderID,
                kBackRightModuleSteerOffsetRadians);

        public static final PIDConstants kAutonTranslationPID = new PIDConstants(4, 0, 0);
        public static final PIDConstants kAutonThetaPID = new PIDConstants(0.5, 0, 0);

        // PID controllers
        public static final PIDController xController = new PIDController(3, 0, 0);
        public static final PIDController yController = new PIDController(3, 0, 0);
        public static final PIDController thetaController = new PIDController(0.2, 0, 0);
        public static final PathConstraints kAutoPathConstraints = new PathConstraints(
                kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
    }
        /**
     * This is code from Poofs 2022
     * 
     * @return the MAC address of the robot
     */
    public static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder ret = new StringBuilder();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                System.out.println("NIS: " + nis.getDisplayName());
                if (nis != null && "eth0".equals(nis.getDisplayName())) {
                    byte[] mac = nis.getHardwareAddress();
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            ret.append(String.format("%02X%s", mac[i],
                                    (i < mac.length - 1) ? ":" : ""));
                        }
                        String addr = ret.toString();
                        System.out.println("NIS " + nis.getDisplayName() + " addr: " + addr);
                        return addr;
                    } else {
                        System.out.println("Address doesn't exist or is not accessible");
                    }
                } else {
                    System.out.println("Skipping adaptor: " + nis.getDisplayName());
                }
            }
        } catch (SocketException | NullPointerException e) {
            e.printStackTrace();
        }
        System.out.println("\n\nMAC ADDRESS NOTHING");
        return "";
    }
}
