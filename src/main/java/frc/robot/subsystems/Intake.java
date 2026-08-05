package frc.robot.subsystems;

//bore encoder imports
//import com.ctre.phoenix6.configs.CANcoderConfiguration;
//import com.ctre.phoenix6.configs.FeedbackConfigs;
//import com.ctre.phoenix6.signals.SensorDirectionValue;
//import com.ctre.phoenix6.hardware.CANcoder;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private boolean intakeRunning = false;
    //Declare variables
    private TalonFX m_PivotMotor;
    private TalonFX m_RollerMotor;

    public boolean m_RunZeroFunction = false;

    //private CANcoder m_ThroughBoreEncoder;
    //private final double canCoderZero = IntakeConstants.kCanCoderZero; // Set this to the absolute encoder reading when the intake is in the stowed position

    private VelocityVoltage velocityControl;

    private final NeutralOut neutralOut = new NeutralOut();

    public Intake () {
        //Configure motor variables
        m_PivotMotor = new TalonFX(IntakeConstants.kPivotMotorId);
        m_RollerMotor = new TalonFX(IntakeConstants.kRollerIntakeMotorId);

        //Setup CanCoder/Configuration
        /*m_ThroughBoreEncoder = new CANcoder(IntakeConstants.kAbsEncoderId);
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.MagnetOffset = canCoderZero;
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; // Or CounterClockwise_Positive
        m_ThroughBoreEncoder.getConfigurator().apply(canCoderConfig);*/

        //Pivot Motor Configuration
        CurrentLimitsConfigs pivotLimits = new CurrentLimitsConfigs();
        pivotLimits.SupplyCurrentLimitEnable = true;
        pivotLimits.SupplyCurrentLimit = IntakeConstants.kPivotCurrentLimit;
        pivotLimits.StatorCurrentLimit = IntakeConstants.kPivotStatorCurrentLimit;
        m_PivotMotor.getConfigurator().apply(pivotLimits);

        TalonFXConfiguration PivotConfigs = new TalonFXConfiguration();
        PivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        PivotConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        Slot0Configs intakeGains = PivotConfigs.Slot0;
        intakeGains.kG = 0.01;//0.1; // Add 0.5 V output to overcome gravity at the pivot's center of mass
        intakeGains.kP = 1;//0.75;//27.5;//20 // A position error of 2.5 rotations results in 12 V output
        intakeGains.kI = 0; // no output for integrated error
        intakeGains.kD = 0.1; //2; // A velocity error of 1 rps results in 0.1 V output

        /*MotionMagicConfigs pivotMotionMagic = PivotConfigs.MotionMagic;
        pivotMotionMagic.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        pivotMotionMagic.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        pivotMotionMagic.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)*/

        //Setup the motor to use the cancoder
        /*FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.FeedbackSensorSource = com.ctre.phoenix6.signals.FeedbackSensorSourceValue.RemoteCANcoder;
        feedbackConfigs.FeedbackRemoteSensorID = m_ThroughBoreEncoder.getDeviceID();
        feedbackConfigs.SensorToMechanismRatio = 1.0;
        feedbackConfigs.RotorToSensorRatio = 60.0;
        PivotConfigs.Feedback = feedbackConfigs;*/

        m_PivotMotor.getConfigurator().apply(PivotConfigs);
        m_PivotMotor.setPosition(0);
        
        //Roller Motor Configuration
        CurrentLimitsConfigs rollerLimits = new CurrentLimitsConfigs();
        rollerLimits.SupplyCurrentLimitEnable = true;
        rollerLimits.SupplyCurrentLimit = IntakeConstants.kRollerCurrentLimit;
        m_RollerMotor.getConfigurator().apply(rollerLimits);

    TalonFXConfiguration RollerConfigs = new TalonFXConfiguration();
    // Use brake mode so the roller resists motion when neutral/stopped
    RollerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        RollerConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        Slot0Configs RollerGains = RollerConfigs.Slot0;
        RollerGains.kS = 0.1; // Ad-21.d 0.25 V output to overcome static friction
        RollerGains.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        RollerGains.kP = 0.11; // An error of 1 rps results in 0.11 V output
        RollerGains.kI = 0; // no output for integrated error
        RollerGains.kD = 0; // no output for error derivative

        /*MotionMagicConfigs rollerMotionMagic = RollerConfigs.MotionMagic;
        rollerMotionMagic.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
        rollerMotionMagic.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)*/

        m_RollerMotor.getConfigurator().apply(RollerConfigs);

    }

  @Override
  public void periodic() {
    if(Constants.kVerboseDashboard)
        SmartDashboard.putNumber("Pivot Position", m_PivotMotor.getPosition().getValueAsDouble());
  }
    public void setPivotPosition(double positionRotations) {
         final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
         m_PivotMotor.setControl(positionRequest.withPosition(positionRotations));
    }
        
    public void stowIntake() {
        setPivotPosition(IntakeConstants.kPivotStowed);
    }



    public void SetPower(double power)
    {
      m_PivotMotor.set(power);
    }


    public void deployIntake() {
        setPivotPosition(IntakeConstants.kPivotDeployed);
    }

    public void agitateIntake() {
        setPivotPosition(IntakeConstants.kPivotAgitate);
    }
        
    public void setRPM(double RPM) {
         velocityControl = new VelocityVoltage(0).withSlot(0);
         m_RollerMotor.setControl(velocityControl.withVelocity(RPM/60));
    }
        
    public void startIntake() {
        //deployIntake();
        setRPM(IntakeConstants.kSpeed);
        intakeRunning = true;
    }
        
    public void reverseIntake() {
        setRPM(IntakeConstants.kSpeed * -1);
        intakeRunning = true;
    }
        
    public void stopIntake() {
        //stowIntake();
        m_RollerMotor.setControl(neutralOut);
        intakeRunning = false;
    }

    public boolean isIntakeRunning() {
        return intakeRunning;
    }

 }