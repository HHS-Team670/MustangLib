package frc.team670.mustanglib.subsystems.drivebase;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.mustanglib.swervelib.ctre.CTRModuleConfiguration;
import frc.team670.mustanglib.swervelib.ctre.CTRSwerveModule;

public class CTRSwerve {
    private final int MODULE_COUNT;

    private Config kConfig;
    private CTRSwerveModule[] m_modules;
    private CTRModuleConfiguration[] m_moduleConfigurations;
    private Pigeon2 m_pigeon2;
    private SwerveDriveKinematics m_kinematics;
    private SwerveDriveOdometry m_odometry;
    private SwerveModulePosition[] m_modulePositions;
    private Translation2d[] m_moduleLocations;
    private OdometryThread m_odometryThread;
    private Field2d m_field;
    private PIDController m_turnPid;
    private Notifier m_telemetry;
    private final CTRModuleConfiguration kModuleConfigFrontLeft = new CTRModuleConfiguration();
    private final CTRModuleConfiguration kModuleConfigFrontRight = new CTRModuleConfiguration();
    private final CTRModuleConfiguration kModuleConfigBackLeft = new CTRModuleConfiguration();
    private final CTRModuleConfiguration kModuleConfigBackRight = new CTRModuleConfiguration();

    /* Put smartdashboard calls in separate thread to reduce performance impact */
    private void telemeterize() {
        SmartDashboard.putNumber("Successful Daqs", m_odometryThread.getSuccessfulDaqs());
        SmartDashboard.putNumber("Failed Daqs", m_odometryThread.getFailedDaqs());
        SmartDashboard.putNumber("X Pos", m_odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Y Pos", m_odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Angle", m_odometry.getPoseMeters().getRotation().getDegrees());
        SmartDashboard.putNumber("Odometry Loop Time", m_odometryThread.getTime());
    }

    /* Perform swerve module updates in a separate thread to minimize latency */
    private class OdometryThread extends Thread {
        private BaseStatusSignal[] m_allSignals;
        public int SuccessfulDaqs = 0;
        public int FailedDaqs = 0;

        private LinearFilter lowpass = LinearFilter.movingAverage(50);
        private double lastTime = 0;
        private double currentTime = 0;
        private double averageLoopTime = 0;

        public OdometryThread() {
            super();
            // 4 signals for each module + 2 for Pigeon2
            m_allSignals = new BaseStatusSignal[(MODULE_COUNT * 4) + 2];
            for (int i = 0; i < MODULE_COUNT; ++i) {
                var signals = m_modules[i].getSignals();
                m_allSignals[(i * 4) + 0] = signals[0];
                m_allSignals[(i * 4) + 1] = signals[1];
                m_allSignals[(i * 4) + 2] = signals[2];
                m_allSignals[(i * 4) + 3] = signals[3];
            }
            m_allSignals[m_allSignals.length - 2] = m_pigeon2.getYaw();
            m_allSignals[m_allSignals.length - 1] = m_pigeon2.getAngularVelocityZWorld();
        }

        @Override
        public void run() {
            /* Make sure all signals update at around 250hz */
            for (var sig : m_allSignals) {
                sig.setUpdateFrequency(250);
            }
            /* Run as fast as possible, our signals will control the timing */
            while (true) {
                /* Synchronously wait for all signals in drivetrain */
                var status = BaseStatusSignal.waitForAll(0.1, m_allSignals);
                lastTime = currentTime;
                currentTime = Utils.getCurrentTimeSeconds();
                averageLoopTime = lowpass.calculate(currentTime - lastTime);

                /* Get status of the waitForAll */
                if (status.isOK()) {
                    SuccessfulDaqs++;
                } else {
                    FailedDaqs++;
                }

                /* Now update odometry */
                for (int i = 0; i < MODULE_COUNT; ++i) {
                    /* No need to refresh since it's automatically refreshed from the waitForAll() */
                    m_modulePositions[i] = m_modules[i].getPosition(false);
                }
                // Assume Pigeon2 is flat-and-level so latency compensation can be performed
                double yawDegrees =
                        BaseStatusSignal.getLatencyCompensatedValue(
                                m_pigeon2.getYaw(), m_pigeon2.getAngularVelocityZWorld());

                m_odometry.update(Rotation2d.fromDegrees(yawDegrees), m_modulePositions);
                m_field.setRobotPose(m_odometry.getPoseMeters());
            }
        }
        
        public double getTime() {
            return averageLoopTime;
        }

        public int getSuccessfulDaqs() {
            return SuccessfulDaqs;
        }

        public int getFailedDaqs() {
            return FailedDaqs;
        }
    }
    public static record Config(double TurnKp, double TurnKd, 
            double kDriveBaseTrackWidth, double kDriveBaseWheelBase, 
            double swerveModuleDriveRatio, double swerveModuleSteerRatio,
            double swerveModuleWheelRadius, double swerveModuleSlipCurrent,
            Slot0Configs swerveModuleSteerGains, Slot0Configs swerveModuleDriveGains,
            boolean SteerMotorReversed, String CANbusName,

            int Pigeon2Id,

            int kFrontLeftModuleDriveMotor, int kFrontLeftModuleSteerMotor,
            int kFrontLeftModuleSteerEncoder, double kFrontLeftEncoderOffset,
            double kFrontLeftLocationX, double kFrontLeftLocationY, 
            
            int kFrontRightModuleDriveMotor, int kFrontRightModuleSteerMotor,
            int kFrontRightModuleSteerEncoder, double kFrontRightEncoderOffset,
            double kFrontRightLocationX, double kFrontRightLocationY,

            int kBackLeftModuleDriveMotor, int kBackLeftModuleSteerMotor,
            int kBackLeftModuleSteerEncoder, double kBackLeftEncoderOffset,
            double kBackLeftLocationX, double kBackLeftLocationY,

            int kBackRightModuleDriveMotor, int kBackRightModuleSteerMotor,
            int kBackRightModuleSteerEncoder, double kBackRightEncoderOffset,
            double kBackRightLocationX, double kBackRightLocationY

            ) {
    }
    public CTRSwerve(
            Config kConfig) {
        this.kConfig = kConfig;
        m_modules = new CTRSwerveModule[4];
        MODULE_COUNT = m_modules.length;
        
        m_pigeon2 = new Pigeon2(kConfig.Pigeon2Id, kConfig.CANbusName);

        m_modules = new CTRSwerveModule[MODULE_COUNT];
        m_moduleConfigurations = new CTRModuleConfiguration[]{
            kModuleConfigFrontLeft, kModuleConfigFrontRight, 
            kModuleConfigBackLeft, kModuleConfigBackRight
        };
        m_modulePositions = new SwerveModulePosition[MODULE_COUNT];
        m_moduleLocations = new Translation2d[MODULE_COUNT];
        
        kModuleConfigFrontLeft.withSteerMotorId(kConfig.kFrontLeftModuleSteerMotor)
                .withDriveMotorId(kConfig.kFrontLeftModuleDriveMotor)
                .withCANcoderId(kConfig.kFrontLeftModuleSteerEncoder)
                .withCANcoderOffset(kConfig.kFrontLeftEncoderOffset)
                .withLocationX(kConfig.kFrontLeftLocationX)
                .withLocationY(kConfig.kFrontLeftLocationY)
                .withDriveMotorGearRatio(kConfig.swerveModuleDriveRatio)
                .withSteerMotorGearRatio(kConfig.swerveModuleSteerRatio)
                .withWheelRadius(kConfig.swerveModuleWheelRadius)
                .withSlipCurrent(kConfig.swerveModuleSlipCurrent)
                .withSteerMotorGains(kConfig.swerveModuleSteerGains)
                .withDriveMotorGains(kConfig.swerveModuleDriveGains)
                .withSteerMotorReversed(kConfig.SteerMotorReversed);

        kModuleConfigFrontRight.withSteerMotorId(kConfig.kFrontRightModuleSteerMotor)
                .withDriveMotorId(kConfig.kFrontRightModuleDriveMotor)
                .withCANcoderId(kConfig.kFrontRightModuleSteerEncoder)
                .withCANcoderOffset(kConfig.kFrontRightEncoderOffset)
                .withLocationX(kConfig.kFrontRightLocationX)
                .withLocationY(kConfig.kFrontRightLocationY)
                .withDriveMotorGearRatio(kConfig.swerveModuleDriveRatio)
                .withSteerMotorGearRatio(kConfig.swerveModuleSteerRatio)
                .withWheelRadius(kConfig.swerveModuleWheelRadius)
                .withSlipCurrent(kConfig.swerveModuleSlipCurrent)
                .withSteerMotorGains(kConfig.swerveModuleSteerGains)
                .withDriveMotorGains(kConfig.swerveModuleDriveGains)
                .withSteerMotorReversed(kConfig.SteerMotorReversed);

        kModuleConfigBackLeft.withSteerMotorId(kConfig.kBackLeftModuleSteerMotor)
                .withDriveMotorId(kConfig.kBackLeftModuleDriveMotor)
                .withCANcoderId(kConfig.kBackLeftModuleSteerEncoder)
                .withCANcoderOffset(kConfig.kBackLeftEncoderOffset)
                .withLocationX(kConfig.kBackLeftLocationX)
                .withLocationY(kConfig.kBackLeftLocationY)
                .withDriveMotorGearRatio(kConfig.swerveModuleDriveRatio)
                .withSteerMotorGearRatio(kConfig.swerveModuleSteerRatio)
                .withWheelRadius(kConfig.swerveModuleWheelRadius)
                .withSlipCurrent(kConfig.swerveModuleSlipCurrent)
                .withSteerMotorGains(kConfig.swerveModuleSteerGains)
                .withDriveMotorGains(kConfig.swerveModuleDriveGains)
                .withSteerMotorReversed(kConfig.SteerMotorReversed);
        
        kModuleConfigBackRight.withSteerMotorId(kConfig.kBackRightModuleSteerMotor)
                .withDriveMotorId(kConfig.kBackRightModuleDriveMotor)
                .withCANcoderId(kConfig.kBackRightModuleSteerEncoder)
                .withCANcoderOffset(kConfig.kBackRightEncoderOffset)
                .withLocationX(kConfig.kBackRightLocationX)
                .withLocationY(kConfig.kBackRightLocationY)
                .withDriveMotorGearRatio(kConfig.swerveModuleDriveRatio)
                .withSteerMotorGearRatio(kConfig.swerveModuleSteerRatio)
                .withWheelRadius(kConfig.swerveModuleWheelRadius)
                .withSlipCurrent(kConfig.swerveModuleSlipCurrent)
                .withSteerMotorGains(kConfig.swerveModuleSteerGains)
                .withDriveMotorGains(kConfig.swerveModuleDriveGains)
                .withSteerMotorReversed(kConfig.SteerMotorReversed);

        for(int iteration = 0; iteration < MODULE_COUNT; iteration++) {
            m_modules[iteration] = new CTRSwerveModule(m_moduleConfigurations[iteration], kConfig.CANbusName);
            m_moduleLocations[iteration] = new Translation2d(m_moduleConfigurations[iteration].LocationX, m_moduleConfigurations[iteration].LocationY);
            m_modulePositions[iteration] = m_modules[iteration].getPosition(true);
        }
        
        m_kinematics = new SwerveDriveKinematics(m_moduleLocations);
        m_odometry =
                new SwerveDriveOdometry(m_kinematics, m_pigeon2.getRotation2d(), getSwervePositions());
        m_field = new Field2d();
        SmartDashboard.putData("Field", m_field);

        m_turnPid = new PIDController(kConfig.TurnKp, 0, kConfig.TurnKd);
        m_turnPid.enableContinuousInput(-Math.PI, Math.PI);

        m_odometryThread = new OdometryThread();
        m_odometryThread.start();

        m_telemetry = new Notifier(this::telemeterize);
        m_telemetry.startPeriodic(0.1); // Telemeterize every 100ms
    }

    private SwerveModulePosition[] getSwervePositions() {
        return m_modulePositions;
    }

    public void driveRobotCentric(ChassisSpeeds speeds) {
        var swerveStates = m_kinematics.toSwerveModuleStates(speeds);
        for (int i = 0; i < MODULE_COUNT; ++i) {
            m_modules[i].apply(swerveStates[i]);
        }
    }

    public void driveFieldCentric(ChassisSpeeds speeds) {
        var roboCentric = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, m_pigeon2.getRotation2d());
        var swerveStates = m_kinematics.toSwerveModuleStates(roboCentric);
        for (int i = 0; i < MODULE_COUNT; ++i) {
            m_modules[i].apply(swerveStates[i]);
        }
    }

    public void driveFullyFieldCentric(double xSpeeds, double ySpeeds, Rotation2d targetAngle) {
        var currentAngle = m_pigeon2.getRotation2d();
        double rotationalSpeed =
                m_turnPid.calculate(currentAngle.getRadians(), targetAngle.getRadians());

        var roboCentric =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeeds, ySpeeds, rotationalSpeed, m_pigeon2.getRotation2d());
        var swerveStates = m_kinematics.toSwerveModuleStates(roboCentric);
        for (int i = 0; i < MODULE_COUNT; ++i) {
            m_modules[i].apply(swerveStates[i]);
        }
    }

    public void driveStopMotion() {
        /* Point every module toward (0,0) to make it close to a X configuration */
        for (int i = 0; i < MODULE_COUNT; ++i) {
            var angle = m_moduleLocations[i].getAngle();
            m_modules[i].apply(new SwerveModuleState(0, angle));
        }
    }

    public void seedFieldRelative() {
        m_pigeon2.setYaw(0);
    }

    public Pose2d getPoseMeters() {
        return m_odometry.getPoseMeters();
    }

    public double getSuccessfulDaqs() {
        return m_odometryThread.SuccessfulDaqs;
    }

    public double getFailedDaqs() {
        return m_odometryThread.FailedDaqs;
    }
}