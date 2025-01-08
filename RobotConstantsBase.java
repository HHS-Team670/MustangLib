package frc.team670.mustanglib;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
/**
 * A place for storing constants required in mustanglib. 
 * This includes most drivebase constants. For every robot, add mac address like below as well as relevant constants
 * 
 * Update every year
 */
public class RobotConstantsBase {
     
    public  static class SwerveDriveBase {
        
        //Front Left
        public static final int kFrontLeftModuleSteerMotorID = 20;
        public static final int kFrontLeftModuleDriveMotorID = 21;
        public static final int kFrontLeftModuleSteerEncoderID = 30;
        //Front Right
        public static final int kFrontRightModuleSteerMotorID = 22;
        public static final int kFrontRightModuleDriveMotorID = 23;
        public static final int kFrontRightModuleSteerEncoderID = 32;
       //Back Right
        public static final int kBackRightModuleSteerMotorID = 24;
        public static final int kBackRightModuleDriveMotorID = 25;
        public static final int kBackRightModuleSteerEncoderID = 34;
        //Back Left
        public static final int kBackLeftModuleSteerMotorID = 26;
        public static final int kBackLeftModuleDriveMotorID = 27;
        public static final int kBackLeftModuleSteerEncoderID = 36;
        
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 8;//What if we turn this up?  //Not robot specific
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI * 16;// Not robot specific

        public static final double kMaxVoltage = 12.0;// Good defaults
        public static final double kMaxDriveCurrent = 60.0;//Defaults
        public static final double kMaxSteerCurrent = 20.0;//Defaults

        public static final double kCurrentSampleTimeRange = 5;
        public static final int kCurrentSampleSize = (int) (kCurrentSampleTimeRange * 50); // mustangPeriodic runs 50hz
        public static final double kTotalDriveCurrentThreshold = 120; // 30 per motor
        public static final double kReducedDriveCurrentLimit = 20; // single motor 

        // The formula for calculating the theoretical maximum velocity is:
        // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
        // pi
        // An example of this constant for a Mk4 L2 module with NEOs to drive is:
        // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
        // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
        
        // Good defaults
        public static final PIDConstants kAutonTranslationPID = new PIDConstants(4, 0, 0);
        public static final PIDConstants kAutonThetaPID = new PIDConstants(1.75, 0, 0);

        // PID controllers
        public static final PIDController xController = new PIDController(3, 0, 0);
        public static final PIDController yController = new PIDController(3, 0, 0);
        public static final PIDController thetaController = new PIDController(0.2, 0, 0);
      
    }

    public static class CTRSwerveDrive {
        
        //Front Left
        public static final int kFrontLeftModuleSteerMotorID = 20;
        public static final int kFrontLeftModuleDriveMotorID = 21;
        public static final int kFrontLeftModuleSteerEncoderID = 30;
        public static final double kFrontLeftEncoderOffset = 0;
        public static final double kFrontLeftLocationX = 0;
        public static final double kFrontLeftLocationY = 0;
        //Front Right
        public static final int kFrontRightModuleSteerMotorID = 22;
        public static final int kFrontRightModuleDriveMotorID = 23;
        public static final int kFrontRightModuleSteerEncoderID = 32;
        public static final double kFrontRightEncoderOffset = 0;
        public static final double kFrontRightLocationX = 0;
        public static final double kFrontRightLocationY = 0;
       //Back Right
        public static final int kBackRightModuleSteerMotorID = 24;
        public static final int kBackRightModuleDriveMotorID = 25;
        public static final int kBackRightModuleSteerEncoderID = 34;
        public static final double kBackRightEncoderOffset = 0;
        public static final double kBackRightLocationX = 0;
        public static final double kBackRightLocationY = 0;
        //Back Left
        public static final int kBackLeftModuleSteerMotorID = 26;
        public static final int kBackLeftModuleDriveMotorID = 27;
        public static final int kBackLeftModuleSteerEncoderID = 36;
        public static final double kBackLeftEncoderOffset = 0;
        public static final double kBackLeftLocationX = 0;
        public static final double kBackLeftLocationY = 0;
        
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 8;//What if we turn this up?  //Not robot specific
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI * 16;// Not robot specific

        public static final double kMaxVoltage = 12.0;// Good defaults
        public static final double kMaxDriveCurrent = 60.0;//Defaults
        public static final double kMaxSteerCurrent = 20.0;//Defaults

        public static final double kCurrentSampleTimeRange = 5;
        public static final int kCurrentSampleSize = (int) (kCurrentSampleTimeRange * 50); // mustangPeriodic runs 50hz
        public static final double kTotalDriveCurrentThreshold = 120; // 30 per motor
        public static final double kReducedDriveCurrentLimit = 20; // single motor 

        public static final Slot0Configs kSteerGains = new Slot0Configs();
        public static final Slot0Configs kDriveGains = new Slot0Configs();
        {
                kSteerGains.kP = 30;
                kSteerGains.kD = 0.2;
                kDriveGains.kP = 1;
        }

        public static final double kDriveRatio = 10; // 10:1 ratio for the drive motor
        public static final double kSteerRatio = 12.8; // 12.8:1 ratio for the steer motor
        public static final double kWheelRadius = 3; // 3 inch radius for the wheels
        public static final double kSlipCurrent = 17; // Only apply 17 stator amps to prevent slip
        public static final boolean kSteerMotorInverted = false; // CANcoder not reversed from the steer motor. For WCP Swerve X this should be true.

        public static final double kTurnD = 1;
        public static final double kTurnP = 1;
        
        public static final int kPigeon2ID = 1;
        public static final String kCANbusname = "Fred";
        // The formula for calculating the theoretical maximum velocity is:
        // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
        // pi
        // An example of this constant for a Mk4 L2 module with NEOs to drive is:
        // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
        // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI

        // Good defaults
        public static final PIDConstants kAutonTranslationPID = new PIDConstants(4, 0, 0);
        public static final PIDConstants kAutonThetaPID = new PIDConstants(1.75, 0, 0);

        // PID controllers
        public static final PIDController xController = new PIDController(3, 0, 0);
        public static final PIDController yController = new PIDController(3, 0, 0);
        public static final PIDController thetaController = new PIDController(0.2, 0, 0);
      
    }

    public static  class TankDriveBase{
        public static final int kLeftLeaderSparkMotorID = 21; 
        public static final int kLeftFollowerSparkMotorID = 22;
        public static final int kRightLeaderSparkMotorID = 23;
        public static final int kRightFollowerSparkMotorID = 24;
        /**
       * The number of ticks per rotation of a drivebase wheel for the SPARK Encoders
       */
        public static final int kSparkTicksPerRotation = 1024;//Not RS
    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = .7;
    
      

  }

    /**
     * This is code from Poofs 2022
     * https://github.com/Team254/FRC-2022-Public/blob/6a24236b37f0fcb75ceb9d5dec767be58ea903c0/src/main/java/com/team254/frc2022/Constants.java#L429 
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