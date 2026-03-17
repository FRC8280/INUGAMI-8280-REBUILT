package frc.robot.subsystems;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.IndexerConstants;
import edu.wpi.first.math.filter.Debouncer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    
    private TalonFX m_IndexerMotor;
    private VelocityVoltage velocityRequest;
    private final NeutralOut neutralOut = new NeutralOut();

    private final CANrange rangeSensor = new CANrange(45);
    private final Debouncer emptyDebounce = new Debouncer(IndexerConstants.kDebounceSeconds);

    public Indexer () {
        m_IndexerMotor = new TalonFX(IndexerConstants.kIndexerMotorId);

        CurrentLimitsConfigs indexerLimits = new CurrentLimitsConfigs();
        indexerLimits.SupplyCurrentLimitEnable = true;
        indexerLimits.SupplyCurrentLimit = IndexerConstants.kIndexerCurrentLimit;
        m_IndexerMotor.getConfigurator().apply(indexerLimits);

        TalonFXConfiguration IndexerConfigs = new TalonFXConfiguration();
        IndexerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        IndexerConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        Slot0Configs indexerGains = IndexerConfigs.Slot0;
        indexerGains.kS = IndexerConstants.kS; // Add 0.25 V output to overcome static friction
        indexerGains.kV = IndexerConstants.kV; // A velocity target of 1 rps results in 0.12 V output
        indexerGains.kP = IndexerConstants.kP; // An error of 1 rps results in 0.11 V output
        indexerGains.kI = IndexerConstants.kI; // no output for integrated error
        indexerGains.kD = IndexerConstants.kD; // no output for error derivative
        
        /*MotionMagicConfigs indexerMotionMagic = IndexerConfigs.MotionMagic;
        indexerMotionMagic.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
        indexerMotionMagic.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)*/

        m_IndexerMotor.getConfigurator().apply(IndexerConfigs);
    }

    public boolean isHopperEmpty() {
        double distance = rangeSensor.getDistance().getValue().in(edu.wpi.first.units.Units.Meters);
        return emptyDebounce.calculate(distance >= IndexerConstants.kEmptyDistance);
    }
    
    public void setRPM(double RPM) {
        velocityRequest = new VelocityVoltage(0).withSlot(0);
        m_IndexerMotor.setControl(velocityRequest.withVelocity(RPM/60));
    }
    public void startIndexer() {
        setRPM(IndexerConstants.kSpeed);
    }
    public void reverseIndexer() {
        setRPM(IndexerConstants.kSpeed * -1);
    }
    public void stopIndexer() {
        m_IndexerMotor.setControl(neutralOut);
    }
}
