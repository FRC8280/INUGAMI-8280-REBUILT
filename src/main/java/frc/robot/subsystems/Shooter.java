package frc.robot.subsystems;

import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
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
import com.ctre.phoenix6.controls.NeutralOut;

public class Shooter extends SubsystemBase{

    // Shooter status state machine
    public enum ShooterStatus { IDLE, WARMUP, FIRING }
    private volatile ShooterStatus m_status = ShooterStatus.IDLE;

    // Warmup timer and duration (seconds)
    private final Timer m_warmupTimer = new Timer();
    
    //Declare variables
    private final Supplier<Pose2d> m_robotPoseSupplier;

    private TalonFX m_ShooterMotor;
    private TalonFX m_FollowerMotor;
    private TalonFX m_AccelerateMotor;
    private ShooterLookup m_ShooterLookup;

    double m_shooterTargetRPM = 0;
    double m_acceleratorTargetRPM = 0;
    boolean m_lineOfSite = false;

    private VelocityVoltage velocityRequest;

    ServoHubConfig config;
    public final HoodSystem m_Hood;
    private final BooleanSubscriber labShooter;
    private final NeutralOut neutralOut = new NeutralOut();
    private final Supplier<LEDSubsystem> m_ledSupplier;
    private double lastLoopTime = Timer.getFPGATimestamp();

    public Shooter(Supplier<Pose2d> robotPoseSupplier, Supplier<LEDSubsystem> ledSupplier) {

        m_robotPoseSupplier = robotPoseSupplier;
        m_ledSupplier = ledSupplier;

        m_Hood = new HoodSystem();
        m_ShooterLookup = new ShooterLookup();
        velocityRequest = new VelocityVoltage(0).withSlot(0);

        labShooter = NetworkTableInstance.getDefault()
            .getBooleanTopic("/Elastic/EnableShooter")
            .subscribe(false);

        m_ShooterMotor = new TalonFX(ShooterConstants.kShooterMotorId);
        m_FollowerMotor = new TalonFX(ShooterConstants.kFollowerMotorId);
        m_AccelerateMotor = new TalonFX(ShooterConstants.kAccelerateMotorId);

        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        
        shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        CurrentLimitsConfigs shooterLimits = shooterConfig.CurrentLimits;
        shooterLimits.SupplyCurrentLimitEnable = true;
        shooterLimits.SupplyCurrentLimit = ShooterConstants.kShooterCurrentLimit;

        Slot0Configs shooterGains = shooterConfig.Slot0;
        shooterGains.kS = ShooterConstants.kS; // Add 0.1 V output to overcome static friction
        shooterGains.kV = ShooterConstants.kV; // A velocity target of 1 rps results in 0.12 V output
        shooterGains.kP = ShooterConstants.kP; // An error of 1 rps results in 0.11 V output
        shooterGains.kI = ShooterConstants.kI; // no output for integrated error
        shooterGains.kD = ShooterConstants.kD; // no output for error derivative

        m_ShooterMotor.getConfigurator().apply(shooterConfig);
        m_FollowerMotor.getConfigurator().apply(shooterConfig); 
        m_FollowerMotor.setControl(new Follower(m_ShooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        //Setup preshot acceleration stage
        TalonFXConfiguration accelConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs accelLimits = accelConfig.CurrentLimits;
        accelLimits.SupplyCurrentLimitEnable = true;
        accelLimits.SupplyCurrentLimit = ShooterConstants.kAccelerateCurrentLimit;
        accelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        accelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        Slot0Configs accelGains = accelConfig.Slot0;
        accelGains.kS = ShooterConstants.kAccelS; // Add 0.1 V output to overcome static friction
        accelGains.kV = ShooterConstants.kAccelV; // A velocity target of 1 rps results in 0.12 V output
        accelGains.kP = ShooterConstants.kAccelP; // An error of 1 rps results in 0.11 V output
        accelGains.kI = ShooterConstants.kAccelI; // no output for integrated error
        accelGains.kD = ShooterConstants.kAccelD; // no output for error derivative

        m_AccelerateMotor.getConfigurator().apply(accelConfig);

    }

    public ShooterStatus getStatus() { return m_status;}
    public void setStatus(ShooterStatus status) {m_status = status;}
    public boolean IsWarmingUp() { return m_status == ShooterStatus.WARMUP;}
    public boolean IsReadyToFire() { return m_status == ShooterStatus.FIRING;}
    public boolean IsIdle() { return m_status == ShooterStatus.IDLE;}
    
    public void WarmupShooter() {
        setStatus(ShooterStatus.WARMUP);
        setShooterVelocity(m_shooterTargetRPM); 
        SetPreShotVelocity(2500); //3000
        
        m_warmupTimer.reset();
        m_warmupTimer.start();
    }

    public void CeaseFire() {
        setStatus(ShooterStatus.IDLE);
        setShooterVelocity(0);
        SetPreShotVelocity(0);
        m_Hood.setAngle(0); 
        m_warmupTimer.stop();
    }


    public void setShooterVelocity(double velocity) {
        m_shooterTargetRPM = velocity;
        if(velocity >0)
            m_ShooterMotor.setControl(velocityRequest.withVelocity(velocity/60));
        else
            m_ShooterMotor.setControl(neutralOut);
    }

    public void SetPreShotVelocity(double velocity)
    {
        //double velocity = 50*60; // RPM for testing, adjust as needed
        m_acceleratorTargetRPM = velocity;

        if(velocity > 0)
            m_AccelerateMotor.setControl(velocityRequest.withVelocity(velocity/60));
        else
            m_AccelerateMotor.setControl(neutralOut);
    }
    
     public static boolean canSeeAllianceTag(String limelightName) {

        Optional<DriverStation.Alliance> allianceOpt = DriverStation.getAlliance();
        if (allianceOpt.isEmpty()) {
            return false;
        }

        LimelightHelpers.RawFiducial[] fiducials =
            LimelightHelpers.getRawFiducials(limelightName);
       
        if(fiducials == null || fiducials.length == 0){
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
    //Todo hook these up to limelight helpers. 
    public boolean hasTarget() { /* vision validity */ return true; }
    public double getRangeMeters() 
    { 

        //Todo - get this from vision / range sensor / pose math
        return 2.73;
    }

    public Translation2d GetAllianceHub()
    {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if(alliance == Alliance.Blue)
            return ShooterConstants.BlueHub;
        else 
            return ShooterConstants.RedHub;
    }

    public Rotation2d getRotationToPoint(Translation2d target) {
        
        Pose2d robotPose = m_robotPoseSupplier.get();
        Translation2d robotPosition = robotPose.getTranslation();
        Translation2d delta = target.minus(robotPosition); // Vector from robot to target
        Rotation2d targetAngle = delta.getAngle(); // Absolute field angle to the target

        return targetAngle.minus(robotPose.getRotation()); // Difference between where robot is pointing and where it should point

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


    public enum AllianceZone { BLUE, CENTER, RED, UNKNOWN }

     /**
     * Returns which alliance zone the robot is in, based on the robot pose from the supplier.
     *
     * Logic:
     *  - Uses the known blue/red hub X coordinates to compute the field midpoint.
     *  - If the robot X is left of (midpoint - halfWidth) -> BLUE
     *  - If the robot X is right of (midpoint + halfWidth) -> RED
     *  - Otherwise -> CENTER
     *
     * @param centerHalfWidthMeters half-width of the center zone in meters (default 1.0 m if using overload)
     * @return AllianceZone enum indicating BLUE, CENTER, RED, or UNKNOWN if pose is unavailable
     */
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

        if (alliance == Alliance.Blue && zone == AllianceZone.BLUE) return true;
        if (alliance == Alliance.Red  && zone == AllianceZone.RED)  return true;
        return false;
    }


    private boolean runThisCycle = false;

    public void periodic(){

        double now = Timer.getFPGATimestamp();
        double loopTime = now - lastLoopTime;
        lastLoopTime = now;

        SmartDashboard.putNumber("Robot/ShooterLoopTime", loopTime);

        double rangeMeters = -1.0;
        double targetAngle = -1.0;
        double hoodActuatorAngle = 0;

        rangeMeters = distanceToAllianceHub();
        ShooterLookup.ShooterSetpoint sp = m_ShooterLookup.getSetpoint(rangeMeters);
        m_shooterTargetRPM = sp.rpm();
        targetAngle = sp.hoodDeg();
        hoodActuatorAngle = ShooterConstants.kShooterDefaultAngle-targetAngle;
                
        // Check warmup timer and transition to FIRING when elapsed
        if (m_status == ShooterStatus.WARMUP) {
            
            if(isInOwnAllianceZone()) {m_Hood.setAngle(hoodActuatorAngle); }

            double elapsed = m_warmupTimer.get();
            if (elapsed >= ShooterConstants.kWarmupSeconds) {
                // warmup complete -> enter FIRING state
                setStatus(ShooterStatus.FIRING);
                m_warmupTimer.stop();
            }
            else
            {
                if( (m_ShooterMotor.getVelocity().getValueAsDouble()*60 >= m_shooterTargetRPM * 0.9) && 
                    (m_AccelerateMotor.getVelocity().getValueAsDouble()*60 >= m_acceleratorTargetRPM * 0.9) ) {
                        // If both shooter and accelerator are at least 90% up to speed, we can transition to FIRING early
                        setStatus(ShooterStatus.FIRING);
                        m_warmupTimer.stop();
                    }
            }
        } else if(m_status != ShooterStatus.FIRING)
            m_Hood.setAngle(0); 

        runThisCycle = !runThisCycle;
        if(!runThisCycle) {
            return; // Skip this cycle to reduce load (adjust as needed)
        }

        m_lineOfSite = canSeeAllianceTag("limelight");
        //todo add case for limelight-rear
        if (m_lineOfSite) 
            m_ledSupplier.get().setGreen();
        else 
            m_ledSupplier.get().setRed();
        
            if(Constants.kVerboseDashboard) {
                SmartDashboard.putNumber("Shooter/Hood Actuator Angle", hoodActuatorAngle);
                SmartDashboard.putBoolean("Shooter/LineOfSight", m_lineOfSite);
                SmartDashboard.putBoolean("Shooter/HasTarget", hasTarget());
                SmartDashboard.putNumber("Shooter/Distance Target", rangeMeters);
                SmartDashboard.putNumber("Shooter/TargetAngle", targetAngle);
                SmartDashboard.putNumber("Shooter/TargetRPM", m_shooterTargetRPM);
                SmartDashboard.putNumber("Shooter/ActualRPM", m_ShooterMotor.getVelocity().getValueAsDouble()*60); 
                SmartDashboard.putNumber("Shooter/AcceleratorTargetRPM", m_acceleratorTargetRPM);
                SmartDashboard.putNumber("Shooter/AcceleratorActualRPM", m_AccelerateMotor.getVelocity().getValueAsDouble()*60);
                SmartDashboard.putNumber("Rotation to alliance hub", getRotationToAllianceHub().getDegrees());
            }
    }

    public  Translation2d getPassingPose(int controllerInput) {

        if (controllerInput < 0 || controllerInput > 5) {
            return null;
        }

        boolean isRed = DriverStation.getAlliance()
            .map(alliance -> alliance == DriverStation.Alliance.Red)
            .orElse(false);

        int index ;
        if (controllerInput <= 2) {

            if (isRed) {
                // Red: flip 3 and 5
                index = 5 - controllerInput;
            } else {
                index = 3 + controllerInput;
            }

        } else {

            if (isRed) {
                if(controllerInput == 3)
                    index = 6;
                else if (controllerInput == 4)
                    index = 7;
                else //(controllerInput == 5)
                    index = 8; 
            } else {
                index = controllerInput - 3;
            }
        }
        return ShooterConstants.PASSING_POSES.get(index);
    }

   

}


