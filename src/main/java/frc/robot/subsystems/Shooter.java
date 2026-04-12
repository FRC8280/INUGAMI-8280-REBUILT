package frc.robot.subsystems;

import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.networktables.BooleanSubscriber;
// import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.servohub.config.ServoHubConfig;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.LimelightHelpers;
import frc.util.ShooterLookup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    // Shooter status state machine
    public enum ShooterStatus {
        IDLE,
        PREWARMUP,
        WARMUP,
        FIRING,
        COASTING_TO_IDLE
    }

    private volatile ShooterStatus m_status = ShooterStatus.IDLE;

    // CTRE control slots
    private static final int kShootSlot = 0;
    private static final int kIdleSlot = 1;

    // Idle shooter behavior
    private static final double kIdleShooterRPM = 1500.0;

    /**
     * We do NOT command idle immediately when leaving FIRING/WARMUP.
     * We neutral the motor first, let it coast naturally, then once the wheel
     * falls near idle speed we softly catch it with the idle PID slot.
     */
    private static final double kIdleCaptureRPM = 1650.0;

    // Warmup timer and duration (seconds)
    private final Timer m_warmupTimer = new Timer();

    // Declare variables
    private final Supplier<Pose2d> m_robotPoseSupplier;

    private TalonFX m_ShooterMotor;
    private TalonFX m_FollowerMotor;
    private TalonFX m_AccelerateMotor;
    private ShooterLookup m_ShooterLookup;

    double m_shooterTargetRPM = 0;
    double m_acceleratorTargetRPM = 0;
    boolean m_lineOfSite = false;

    private final VelocityVoltage shooterVelocityRequest =
            new VelocityVoltage(0).withSlot(kShootSlot);
    private final VelocityVoltage shooterIdleVelocityRequest =
            new VelocityVoltage(0).withSlot(kIdleSlot);
    private final VelocityVoltage acceleratorVelocityRequest =
            new VelocityVoltage(0).withSlot(kShootSlot);

    ServoHubConfig config;
    public final HoodSystem m_Hood;
    private final NeutralOut neutralOut = new NeutralOut();
    private final Supplier<LEDSubsystem> m_ledSupplier;
    private double lastLoopTime = Timer.getFPGATimestamp();

    public Shooter(Supplier<Pose2d> robotPoseSupplier, Supplier<LEDSubsystem> ledSupplier) {

        m_robotPoseSupplier = robotPoseSupplier;
        m_ledSupplier = ledSupplier;

        m_Hood = new HoodSystem();
        m_ShooterLookup = new ShooterLookup();

        /*
         * labShooter = NetworkTableInstance.getDefault()
         * .getBooleanTopic("/Elastic/EnableShooter")
         * .subscribe(false);
         */

        m_ShooterMotor = new TalonFX(ShooterConstants.kShooterMotorId);
        m_FollowerMotor = new TalonFX(ShooterConstants.kFollowerMotorId);
        m_AccelerateMotor = new TalonFX(ShooterConstants.kAccelerateMotorId);

        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

        shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        CurrentLimitsConfigs shooterLimits = shooterConfig.CurrentLimits;
        shooterLimits.SupplyCurrentLimitEnable = true;
        shooterLimits.SupplyCurrentLimit = ShooterConstants.kShooterCurrentLimit;

        // Slot 0 = normal shooting PID
        Slot0Configs shooterGains = shooterConfig.Slot0;
        shooterGains.kS = ShooterConstants.kS;
        shooterGains.kV = ShooterConstants.kV;
        shooterGains.kP = ShooterConstants.kP;
        shooterGains.kI = ShooterConstants.kI;
        shooterGains.kD = ShooterConstants.kD;
        shooterGains.kA = ShooterConstants.kA;

        // Slot 1 = idle hold PID, much softer than shoot slot
        Slot1Configs shooterIdleGains = shooterConfig.Slot1;
        shooterIdleGains.kS = ShooterConstants.kS;
        shooterIdleGains.kV = ShooterConstants.kV;
        shooterIdleGains.kP = ShooterConstants.kP * 0.10;
        shooterIdleGains.kI = 0.0;
        shooterIdleGains.kD = 0.0;
        shooterIdleGains.kA = ShooterConstants.kA;

        /*
         * MotionMagicConfigs shooterMotionMagic = shooterConfig.MotionMagic;
         * shooterMotionMagic.MotionMagicCruiseVelocity = 80;
         * shooterMotionMagic.MotionMagicAcceleration = 160;
         * shooterMotionMagic.MotionMagicJerk = 1600;
         */

        m_ShooterMotor.getConfigurator().apply(shooterConfig);
        m_FollowerMotor.getConfigurator().apply(shooterConfig);
        m_FollowerMotor.setControl(new Follower(m_ShooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        // Setup preshot acceleration stage
        TalonFXConfiguration accelConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs accelLimits = accelConfig.CurrentLimits;
        accelLimits.SupplyCurrentLimitEnable = true;
        accelLimits.SupplyCurrentLimit = ShooterConstants.kAccelerateCurrentLimit;
        accelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        accelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        Slot0Configs accelGains = accelConfig.Slot0;
        accelGains.kS = ShooterConstants.kAccelS;
        accelGains.kV = ShooterConstants.kAccelV;
        accelGains.kP = ShooterConstants.kAccelP;
        accelGains.kI = ShooterConstants.kAccelI;
        accelGains.kD = ShooterConstants.kAccelD;
        accelGains.kA = ShooterConstants.kAccelA;

        m_AccelerateMotor.getConfigurator().apply(accelConfig);
    }

    public ShooterStatus getStatus() {
        return m_status;
    }

    public void setStatus(ShooterStatus status) {
        m_status = status;
    }

    public boolean IsWarmingUp() {
        return m_status == ShooterStatus.WARMUP;
    }

    public boolean IsReadyToFire() {
        return m_status == ShooterStatus.FIRING;
    }

    public boolean IsIdle() {
        return m_status == ShooterStatus.IDLE || m_status == ShooterStatus.COASTING_TO_IDLE;
    }

    public void ActivatePreShot() {
        System.out.println("Activate Preshot.");
        setStatus(ShooterStatus.PREWARMUP);
        // SetPreShotVelocity(2000); // 3000
    }

    public void DeactivatePreShot() {
        System.out.println("Deactivate Preshot.");

        if (m_status == ShooterStatus.PREWARMUP || m_status == ShooterStatus.FIRING || m_status == ShooterStatus.WARMUP) {
            beginCoastToIdle();
            return;
        }

        setStatus(ShooterStatus.IDLE);
        SetPreShotVelocity(0);
    }

    public void SetHood(double angle) {
        m_Hood.setAngle(angle);
    }

    public void WarmupShooter() {
        setStatus(ShooterStatus.WARMUP);
        setShooterVelocity(m_shooterTargetRPM);
        SetPreShotVelocity(3000); // 2500

        m_warmupTimer.reset();
        m_warmupTimer.start();
    }

    public void CeaseFire() {
        beginCoastToIdle();
        m_Hood.setAngle(0);
        m_warmupTimer.stop();
    }

    private void beginCoastToIdle() {
        setStatus(ShooterStatus.COASTING_TO_IDLE);

        // Let the shooter coast naturally. No braking, no velocity control yet.
        m_ShooterMotor.setControl(neutralOut);

        // Accelerator is not being held at idle; let it neutral out fully.
        m_AccelerateMotor.setControl(neutralOut);
        m_acceleratorTargetRPM = 0;
    }

    public void engageShooterIdle() {
        m_shooterTargetRPM = kIdleShooterRPM;
        m_ShooterMotor.setControl(
                shooterIdleVelocityRequest.withVelocity(kIdleShooterRPM / 60.0));
        setStatus(ShooterStatus.IDLE);
    }

    public void setShooterVelocity(double velocity) {
        m_shooterTargetRPM = velocity;
        if (velocity > 0) {
            m_ShooterMotor.setControl(shooterVelocityRequest.withVelocity(velocity / 60.0));
        } else {
            m_ShooterMotor.setControl(neutralOut);
        }
    }

    public void SetPreShotVelocity(double velocity) {
        m_acceleratorTargetRPM = velocity;

        if (velocity > 0) {
            m_AccelerateMotor.setControl(acceleratorVelocityRequest.withVelocity(velocity / 60.0));
        } else {
            m_AccelerateMotor.setControl(neutralOut);
        }
    }

    public static boolean canSeeAllianceTag(String limelightName) {

        Optional<DriverStation.Alliance> allianceOpt = DriverStation.getAlliance();
        if (allianceOpt.isEmpty()) {
            return false;
        }

        LimelightHelpers.RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(limelightName);

        if (fiducials == null || fiducials.length == 0) {
            return false;
        }

        Set<Integer> validTags = (allianceOpt.get() == DriverStation.Alliance.Blue)
                ? ShooterConstants.BLUE_TAGS
                : ShooterConstants.RED_TAGS;

        for (LimelightHelpers.RawFiducial f : fiducials) {
            if (validTags.contains(f.id))
                return true;
        }

        return false;
    }

    // Todo hook these up to limelight helpers.
    public boolean hasTarget() {
        return true;
    }

    public double getRangeMeters() {
        // Todo - get this from vision / range sensor / pose math
        return 2.73;
    }

    public Translation2d GetAllianceHub() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if (alliance == Alliance.Blue)
            return ShooterConstants.BlueHub;
        else
            return ShooterConstants.RedHub;
    }

    public Rotation2d getRotationToPoint(Translation2d target) {
        Pose2d robotPose = m_robotPoseSupplier.get();
        Translation2d robotPosition = robotPose.getTranslation();
        Translation2d delta = target.minus(robotPosition);
        Rotation2d targetAngle = delta.getAngle();

        return targetAngle.minus(robotPose.getRotation());
    }

    public Rotation2d getRotationToAllianceHub() {
        return getRotationToPoint(GetAllianceHub());
    }

    public double distanceToAllianceHub() {
        Pose2d pose = m_robotPoseSupplier.get();
        if (pose == null) {
            return Double.NaN;
        }

        Translation2d hubPoint = GetAllianceHub();
        return pose.getTranslation().getDistance(hubPoint);
    }

    public enum AllianceZone {
        BLUE, CENTER, RED, UNKNOWN
    }

    public AllianceZone getAllianceZone() {
        Pose2d pose = m_robotPoseSupplier.get();
        if (pose == null) {
            return AllianceZone.UNKNOWN;
        }

        final double blueX = 4.632;
        final double redX = 11.89;
        double midX = (blueX + redX) / 2.0;

        double robotX = pose.getX();

        if (robotX < midX) {
            return AllianceZone.BLUE;
        } else if (robotX > midX) {
            return AllianceZone.RED;
        } else {
            return AllianceZone.CENTER;
        }
    }

    public boolean isInOwnAllianceZone() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        AllianceZone zone = getAllianceZone();

        if (alliance == Alliance.Blue && zone == AllianceZone.BLUE)
            return true;
        if (alliance == Alliance.Red && zone == AllianceZone.RED)
            return true;
        return false;
    }

    // Passing override: when true, use the fixed passing RPM and hood angle
    // instead of the range-based lookup performed in periodic(). This is
    // enabled by RobotContainer when performing passes between robots.
    private boolean fPassing = false;
    private double m_passingTargetRPM = 0.0; // RPM
    private double m_passingHoodDeg = 0.0;   // hood degrees

    /**
     * Enable passing override: shooter will use provided RPM and hood angle
     * until disablePassingMode() is called.
     */
    public void enablePassingMode(double rpm, double hoodDeg) {
        fPassing = true;
        m_passingTargetRPM = rpm;
        m_passingHoodDeg = hoodDeg;
        SmartDashboard.putBoolean("Shooter/PassingMode", true);
        SmartDashboard.putNumber("Shooter/PassingRPM", rpm);
        SmartDashboard.putNumber("Shooter/PassingHoodDeg", hoodDeg);
    }

    /** Disable passing override and resume normal automatic setpoint selection. */
    public void disablePassingMode() {
        fPassing = false;
        SmartDashboard.putBoolean("Shooter/PassingMode", false);
    }

    private boolean runThisCycle = false;

    @Override
    public void periodic() {

        double now = Timer.getFPGATimestamp();
        double loopTime = now - lastLoopTime;
        lastLoopTime = now;

        SmartDashboard.putNumber("Robot/ShooterLoopTime", loopTime);

        double rangeMeters = -1.0;
        double targetAngle = -1.0;
        double hoodActuatorAngle = 0;

        rangeMeters = distanceToAllianceHub();
        if (fPassing) {
            // Use fixed passing setpoints when in passing mode
            m_shooterTargetRPM = m_passingTargetRPM;
            targetAngle = m_passingHoodDeg;
        } else {
            ShooterLookup.ShooterSetpoint sp = m_ShooterLookup.getSetpoint(rangeMeters);
            m_shooterTargetRPM = sp.rpm();
            targetAngle = sp.hoodDeg();
        }
        hoodActuatorAngle = ShooterConstants.kShooterDefaultAngle - targetAngle;

        double shooterActualRPM = m_ShooterMotor.getVelocity().getValueAsDouble() * 60.0;
        double acceleratorActualRPM = m_AccelerateMotor.getVelocity().getValueAsDouble() * 60.0;

        if (m_status == ShooterStatus.PREWARMUP) {
            // Fixed: setter expects RPM, not RPS
            setShooterVelocity(kIdleShooterRPM);

        } else if (m_status == ShooterStatus.COASTING_TO_IDLE) {
            // Let the wheel coast naturally until it gets near idle speed,
            // then softly catch it with the low-P idle slot.
            if (shooterActualRPM <= kIdleCaptureRPM) {
                engageShooterIdle();
            }

        } else if (m_status == ShooterStatus.WARMUP) {

            if (isInOwnAllianceZone()) {
                m_Hood.setAngle(hoodActuatorAngle);
            }

            double elapsed = m_warmupTimer.get();
            if (elapsed >= ShooterConstants.kWarmupSeconds) {
                setStatus(ShooterStatus.FIRING);
                m_warmupTimer.stop();
            } else {
                if ((shooterActualRPM >= m_shooterTargetRPM * 0.9) &&
                        (acceleratorActualRPM >= m_acceleratorTargetRPM * 0.9)) {
                    setStatus(ShooterStatus.FIRING);
                    m_warmupTimer.stop();
                }
            }
        } else if (m_status != ShooterStatus.FIRING) {
            m_Hood.setAngle(0);
        }

        runThisCycle = !runThisCycle;
        if (!runThisCycle) {
            return;
        }

        m_lineOfSite = canSeeAllianceTag("limelight");

        // todo add case for limelight-rear
        if (m_lineOfSite) {
            if (rangeMeters > 3 && rangeMeters < 3.5) {
                m_ledSupplier.get().setYellow(false);
            } else if (rangeMeters < 3) {
                m_ledSupplier.get().setGreen(false);
            }
        } else {
            m_ledSupplier.get().setRed(false);
        }

        if (Constants.kVerboseDashboard) {
            SmartDashboard.putNumber("Shooter/Hood Actuator Angle", hoodActuatorAngle);
            SmartDashboard.putBoolean("Shooter/LineOfSight", m_lineOfSite);
            SmartDashboard.putBoolean("Shooter/HasTarget", hasTarget());
            SmartDashboard.putNumber("Shooter/Distance Target", rangeMeters);
            SmartDashboard.putNumber("Shooter/TargetAngle", targetAngle);
            SmartDashboard.putNumber("Shooter/TargetRPM", m_shooterTargetRPM);
            SmartDashboard.putNumber("Shooter/ActualRPM", shooterActualRPM);
            SmartDashboard.putNumber("Shooter/AcceleratorTargetRPM", m_acceleratorTargetRPM);
            SmartDashboard.putNumber("Shooter/AcceleratorActualRPM", acceleratorActualRPM);
            SmartDashboard.putNumber("Rotation to alliance hub", getRotationToAllianceHub().getDegrees());
            SmartDashboard.putString("Shooter/State", m_status.name());
        }
    }

    public Translation2d getPassingPoseLeftButton() {

        boolean isRed = DriverStation.getAlliance()
                .map(alliance -> alliance == DriverStation.Alliance.Red)
                .orElse(false);

        if (isRed)
            return ShooterConstants.kRedLeftPass;
        else
            return ShooterConstants.kBlueLeftPass;
    }

    public Translation2d getPassingPoseRightButton() {

        boolean isRed = DriverStation.getAlliance()
                .map(alliance -> alliance == DriverStation.Alliance.Red)
                .orElse(false);

        if (isRed)
            return ShooterConstants.kRedRightPass;
        else
            return ShooterConstants.kBlueRightPass;
    }
}