package frc.team670.mustanglib;

import static java.util.Map.entry;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
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
    public static final String kWestCoastAddress = "Rember to change this";
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
                    entry("kSwerveModuleConfig", 1.0))),
        entry(kWestCoastAddress,
        Map.ofEntries(
                entry("kDriveBaseGearRatio", 10.71),
                entry("kDriveBaseTrackWidth", 24.0),
                entry("kDriveBaseWidthInches",26.0),
                entry("kDriveBaseLengthInches",31.5),
                entry("kWheelDiameterInches", 8.0),
                entry("kDriveBaseDistanceToGroundInches",4.0))))
            .get(kRobotAddress);
            
    public static final class SwerveDriveBase {
        
        public static final double kWidth = Units.inchesToMeters(36);
        public static double kClearance = Math.hypot(kWidth, kWidth) / 2 + 0.05;
        public static final double kTrackWidthMeters = 0.6096;
        public static final double kWheelBaseMeters = 0.6096;

        public static final ModuleConfiguration kModuleConfig = getValue("kSwerveModuleConfig")==-1?null: robotSpecificConstants.get("kSwerveModuleConfig")==1.0
                ? SdsModuleConfigurations.MK4I_L1
                : SdsModuleConfigurations.MK4I_L2;

        public static final GearRatio kSwerveModuleGearRatio = getValue("kSwerveModuleConfig")==-1?null:robotSpecificConstants.get("kSwerveModuleConfig") == 1.0
                ? GearRatio.L1
                : GearRatio.L2;

        public static final int kFrontLeftModuleSteerMotorID = 20;
        public static final int kFrontLeftModuleDriveMotorID = 21;
        public static final int kFrontLeftModuleSteerEncoderID = 30;
        public static final double kFrontLeftModuleSteerOffsetRadians = getValue("kFrontLeftModuleSteerOffsetRadians");
        public static final int kFrontRightModuleSteerMotorID = 22;
        public static final int kFrontRightModuleDriveMotorID = 23;
        public static final int kFrontRightModuleSteerEncoderID = 32;
        public static final double kFrontRightModuleSteerOffsetRadians = getValue("kFrontRightModuleSteerOffsetRadians");

        public static final int kBackLeftModuleSteerMotorID = 26;
        public static final int kBackLeftModuleDriveMotorID = 27;
        public static final int kBackLeftModuleSteerEncoderID = 36;
        public static final double kBackLeftModuleSteerOffsetRadians = getValue("kBackLeftModuleSteerOffsetRadians");

        public static final int kBackRightModuleSteerMotorID = 24;
        public static final int kBackRightModuleDriveMotorID = 25;
        public static final int kBackRightModuleSteerEncoderID = 34;
        public static final double kBackRightModuleSteerOffsetRadians = getValue("kBackRightModuleSteerOffsetRadians");

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
    public static final class TankDriveBase{
        
        // public static final int PDP_ID = 0;

        //   public static final int DRIVER_CONTROLLER_PORT = 0;
        //   public static final int OPERATOR_CONTROLLER_PORT = 1;
        //   public static final int BACKUP_CONTROLLER_PORT = 2;

          // Drive Base
          public static final int SPARK_LEFT_MOTOR_1 = 22; 
          public static final int SPARK_LEFT_MOTOR_2 = 21;
          public static final int SPARK_RIGHT_MOTOR_1 = 23;
          public static final int SPARK_RIGHT_MOTOR_2 = 24;
          public final static SerialPort.Port NAVX_PORT = SerialPort.Port.kUSB;
      /**
       * The number of ticks per rotation of a drivebase wheel for the DIO Encoders
       */
      public static final int DIO_TICKS_PER_ROTATION = 1024;

      /**
       * The number of ticks per rotation of a drivebase wheel for the SPARK Encoders
       */
      public static final int SPARK_TICKS_PER_ROTATION = 1024;
      
      // Drive Base Gearing
      public static final double DRIVEBASE_GEAR_RATIO = getValue("kDriveBaseGearRatio");

      // Drive Wheel Diameter in Inches
      public static final double DRIVE_BASE_WHEEL_DIAMETER = getValue("kWheelDiameterInches");

      // Inches per rotation of the NEO motors on the drivebase
      public static final double DRIVEBASE_INCHES_PER_ROTATION = 
          1 / DRIVEBASE_GEAR_RATIO * DRIVE_BASE_WHEEL_DIAMETER * Math.PI;

      // Number of ticks per inch of wheel travel
      public static final int DIO_TICKS_PER_INCH = 
          (int) (DIO_TICKS_PER_ROTATION / (Math.PI * DRIVE_BASE_WHEEL_DIAMETER));

      // Number of meters per roatation of a drivebase/hdrive wheel
      public static final double DRIVEBASE_METERS_PER_ROTATION = 
          (1 / DRIVEBASE_GEAR_RATIO) * DRIVE_BASE_WHEEL_DIAMETER * Math.PI * 0.0254;


      public static final double DRIVEBASE_VELOCITY_CONVERSION_FACTOR = DRIVEBASE_METERS_PER_ROTATION / 60;

     
      //
      public static final int kTimeoutMs = 0;
      public static final double leftKsVolts = 0.4; //0.20806; //0.4; 
      public static final double leftKvVoltSecondsPerMeter = 2.1; //1.3667; //2.7378; 
      public static final double leftKaVoltSecondsSquaredPerMeter = 0.15; //0.21286; //0.5584; //0.333; 
      public static final double rightKsVolts = leftKsVolts;
      public static final double rightKvVoltSecondsPerMeter = leftKvVoltSecondsPerMeter;
      public static final double rightKaVoltSecondsSquaredPerMeter = leftKaVoltSecondsSquaredPerMeter;

      public static final double kTrackwidthMeters = getValue("kDriveBaseTrackWidth");

      // Autonomous Constants
      public static final DifferentialDriveKinematics kDriveKinematics = 
          new DifferentialDriveKinematics(kTrackwidthMeters);

      public static final double leftKPDriveVel = 2;
      public static final double leftKIDriveVel = 0;
      public static final double leftKDDriveVel = 0;

      public static final double rightKPDriveVel = leftKPDriveVel;
      public static final double rightKIDriveVel = leftKIDriveVel;
      public static final double rightKDDriveVel = leftKDDriveVel;

      public static final double kMaxSpeedInchesPerSecond = 6;
      public static final double kMaxAccelerationInchesPerSecondSquared = 6;

      public static final double kMaxSpeedMetersPerSecond = 2;
      public static final double kMaxAccelerationMetersPerSecondSquared = 2;
      public static final double endVelocityMetersPerSecond = 0;

      public static final double kMaxSpeedMetersPerSecond2 = 0.3;
      public static final double kMaxAccelerationMetersPerSecondSquared2 = 0.3;
      public static final double endVelocityMetersPerSecond2 = 0.2;

      public static final DifferentialDriveKinematicsConstraint kAutoPathConstraints = 
          new DifferentialDriveKinematicsConstraint(kDriveKinematics, kMaxSpeedMetersPerSecond);

      public static final DifferentialDriveKinematicsConstraint kAutoPathConstraintsIntaking = 
          new DifferentialDriveKinematicsConstraint(kDriveKinematics, kMaxSpeedMetersPerSecond);

      // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
      public static final double kRamseteB = 2;
      public static final double kRamseteZeta = .7;
      public static final boolean kNavXReversed = true;
      
      // Robot Dimensions in Inches
      public static final double ROBOT_LENGTH = getValue("kDriveBaseLengthInches"),
                                 ROBOT_WIDTH = getValue("kDriveBaseWidthInches"),
                                 DRIVEBASE_TO_GROUND = getValue("kDriveBaseDistanceToGroundInches"),
                                 ROBOT_FULL_LENGTH_WITH_BUMPER = ROBOT_LENGTH + 6;
  }
  public static double getValue(String key){
     Double value =null;
    try{
        value=robotSpecificConstants.get(key);
    }catch(Exception e){

    }
   
    if(value == null){
        return -1;
    }
    return value;
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
