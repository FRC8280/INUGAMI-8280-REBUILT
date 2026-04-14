package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

/**
 * HoodSystem implemented with a CTRE TalonFX-driven actuator (minion motor).
 *
 * This replaces the previous servo-based implementation. The hood uses
 * positional control (PositionVoltage) to move to target positions. A small
 * interpolation table maps desired hood angles (degrees) to motor position
 * rotations; initially the table contains two points:
 * - angle kHoodAngleA -> motor position kHoodSetpointA (home/start)
 * - angle kHoodAngleB -> motor position kHoodSetpointB (max/lowered)
 *
 * Tune the calibration points in `Constants.HoodConstants` through testing.
 */
public class HoodSystem extends SubsystemBase {

    private final TalonFXS m_hoodMotor;
    private final NeutralOut neutralOut = new NeutralOut();

    // Interpolator mapping from hood angle (deg) -> motor position (rotations)
    private final InterpolatingDoubleTreeMap angleToPosition = new InterpolatingDoubleTreeMap();

    private double m_currentAngleDeg = 0.0;

    public HoodSystem() {
        m_hoodMotor = new TalonFXS(Constants.HoodConstants.kHoodMotorId);

        // Motor configuration
        TalonFXSConfiguration cfg = new TalonFXSConfiguration();
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        cfg.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        CurrentLimitsConfigs limits = cfg.CurrentLimits;
        limits.SupplyCurrentLimitEnable = true;
        limits.SupplyCurrentLimit = Constants.HoodConstants.kHoodSupplyCurrentLimit; // use constant

        // Position PID gains (slot 0)
        Slot0Configs slot0 = cfg.Slot0;
        slot0.kG = Constants.HoodConstants.kHoodKG; // gravity/feedforward if needed
        slot0.kP = Constants.HoodConstants.kHoodKP; // tuned for position control; adjust as needed
        slot0.kI = Constants.HoodConstants.kHoodKI;
        slot0.kD = Constants.HoodConstants.kHoodKD;

        m_hoodMotor.getConfigurator().apply(cfg);

        // Initialize motor reported position to 0 (assumes the mechanism is
        // physically zeroed before enabling)
        m_hoodMotor.setPosition(0);

        // Ensure the motor is neutral with brake enabled (configured above).
        // Also set the output to neutral so the motor holds position safely.
        m_hoodMotor.setControl(neutralOut);

    // Build the simple two-point lookup table. Keys are hood angles (deg),
    // values are motor rotations.
    // Calibration point A (home): hood angle A -> motor rotations A
    angleToPosition.put(Constants.HoodConstants.kHoodAngleA, Constants.HoodConstants.kHoodSetpointA);
    // Calibration point B (max/lowered): hood angle B -> motor rotations B
    angleToPosition.put(Constants.HoodConstants.kHoodAngleB, Constants.HoodConstants.kHoodSetpointB);

        // Publish initial state (only when verbose dashboard enabled)
        if (Constants.kVerboseDashboard) {
            SmartDashboard.putBoolean("Hood/UsingMotor", true);
            SmartDashboard.putNumber("Hood/MapPointB_AngleDeg", Constants.HoodConstants.kHoodAngleB);
            SmartDashboard.putNumber("Hood/MapPointB_Rotations", Constants.HoodConstants.kHoodSetpointB);
            SmartDashboard.putString("Hood/NeutralMode", "Brake");
        }
    }

    @Override
    public void periodic() {
        double motorPos = m_hoodMotor.getPosition().getValueAsDouble();
        if (Constants.kVerboseDashboard) {
            SmartDashboard.putNumber("Hood/MotorPositionRotations", motorPos);
            SmartDashboard.putNumber("Hood/AngleDeg", m_currentAngleDeg);
        }
    }

    /**
     * Set hood to desired angle (degrees). Uses the interpolator to determine
     * the motor position (rotations) to command.
     *
     * @param angleDeg desired hood angle in degrees
     */
    public void setAngle(double angleDeg) {

        // Clamp angle within known interpolation range (use the two calibration
        // points to derive min/max so the clamp follows constants in HoodConstants)
        double minAngle = Math.min(Constants.HoodConstants.kHoodAngleA, Constants.HoodConstants.kHoodAngleB);
        double maxAngle = Math.max(Constants.HoodConstants.kHoodAngleA, Constants.HoodConstants.kHoodAngleB);
        double clamped = Math.max(minAngle, Math.min(maxAngle, angleDeg));

        // InterpolatingDoubleTreeMap.get() will return nearest endpoint if
        // outside range, and interpolate between points otherwise.
        double motorRotations = angleToPosition.get(clamped);

        // Command the motor using position control
        final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
        m_hoodMotor.setControl(positionRequest.withPosition(motorRotations));

        m_currentAngleDeg = clamped;

        if (Constants.kVerboseDashboard) {
            SmartDashboard.putNumber("Hood/CommandedAngleDeg", clamped);
            SmartDashboard.putNumber("Hood/CommandedMotorRotations", motorRotations);
        } 
    }

    /**
     * Add or update a calibration point that maps hood angle (deg) to motor
     * rotations used by the interpolator.
     */
    public void addCalibrationPoint(double angleDeg, double motorRotations) {
        angleToPosition.put(angleDeg, motorRotations);
    }

    /**
     * Return the motor-reported position (rotations).
     */
    public double getMotorPositionRotations() {
        return m_hoodMotor.getPosition().getValueAsDouble();
    }

    /**
     * Return the last commanded hood angle (degrees).
     */
    public double getCommandedAngleDeg() {
        return m_currentAngleDeg;
    }

    /** Stop and neutral the hood motor. */
    public void stop() {
        m_hoodMotor.setControl(neutralOut);
    }
}
