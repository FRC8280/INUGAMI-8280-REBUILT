package frc.robot;

import java.util.Set;

import edu.wpi.first.math.geometry.Translation2d;

public class Constants {

    public enum PassingZone {
        NOT_PASSING,
        Zone_Left,
        Zone_Right
    }

    public static final class LEDConstants {
        public static final int kRearCandleID = 50; // CAN bus ID for the CANdle
        public static final int kFrontCandleID = 51; // CAN bus ID for the timer candle (if used)

    }


    public static final boolean kVerboseDashboard = true;
    public static final boolean kVerboseDashboardBeta = true;
    public static final double kWarmupSeconds = 4.0;
    public static final double kMaxSpeed = 1.0;

    public static final class LinearServoConstants {
        public static final int kServoHubId = 10;
        public static final int kLinearServoLeftChannel = 0;
        public static final int kLinearServoRightChannel = 1;
        public static final int kPulsePeriod = 5000;
        public static final int kMinPulseWidth = 1200;
        public static final int kMidPulseWidth = 1500;
        public static final int kMaxPulseWidth = 2000;  

        public static final int kMaxExtensionPulseWidth = 2000; // Adjust this value based on your servo's specifications
        public static final int kMinExtensionPulseWidth = 1200; // Adjust this value based on your servo's specifications
    }
    
    public static final class IntakeConstants{
        public static final int kPivotMotorId = 20;
        public static final int kPivotCurrentLimit = 30;
        public static final int kPivotStatorCurrentLimit = 60;
        public static final double kPivotStowed = 0;
        public static final double kPivotDeployed = -21.316;
        public static final double kPivotAgitate = -13.6;//-15.650;

        public static final int kRollerIntakeMotorId = 12;
        public static final int kSpeed = -5000; //-5000;
        public static final int kRollerCurrentLimit =40;

        public static final int kAbsEncoderId = 45;
        public static final double kCanCoderZero = 0.128; // Set this to the absolute encoder reading when the intake is in the stowed position
    }

    public static final class IndexerConstants{
        public static final int kIndexerMotorId = 11;
        public static final int kIndexerCurrentLimit = 40;
        public static final int kSpeed = 4500; //5000;

        public static final double kS = 0.25;
        public static final double kV = 0.12;
        public static final double kP = 0.11;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final double kDebounceSeconds = 0.5;
        public static final double kEmptyDistance = 0.6; // Meters, adjust based on your sensor and indexing mechanism
    }

    public static final class ShooterConstants{
        //shooter PID values
        public static final double kP = 0.3;//0.12761;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0.095501;
        public static final double kV = 0.1156;
        public static final double kA = 0.011068;

        //Accelerator PID values (if using)
        public static final double kAccelP = 0.3;//0.014922;
        public static final double kAccelI = 0;
        public static final double kAccelD = 0;
        public static final double kAccelS = 0.3875;
        public static final double kAccelV = 0.097625;
        public static final double kAccelA = 0.0014459;

        public static final double idleRPM = 2000;
    

        public static final int kShooterMotorId = 30;
        public static final int kShooterCurrentLimit = 40;
        public static final int kFollowerMotorId = 31;
    // Additional follower motors (CAN IDs 32 and 33)
    public static final int kFollowerMotorLeft1Id = 32;
    public static final int kFollowerMotorRight1Id = 33;

        public static final int kAccelerateCurrentLimit = 30;
        public static final int kAccelerateMotorId = 16;
        
        public static final int kServoHubId = 10;

        public static final double kShooterTargetVelocity = 240; // RPM
        
    public static final double kHoodMinDeg = 25.0; // Minimum hood angle in degrees
    public static final double kHoodMaxDeg = 70.0; // Maximum hood angle in degrees

        public static final Set<Integer> BLUE_TAGS = Set.of(25,26,21,24,18,27);
        public static final Set<Integer> RED_TAGS = Set.of(9,10,11,12,8,5);

        public static final double kShooterDefaultAngle = 70.0;
        public static final double kWarmupSeconds = 2.0;
        // Preshot accelerator RPM used during WarmupShooter pre-shot stage
       public static final double kPreShotRPM = 3000;

        public static Translation2d kBlueLeftPass = new Translation2d(0.5, 7.5);
        public static Translation2d kBlueRightPass = new Translation2d(0.5, 0.5);
        public static Translation2d kRedLeftPass = new Translation2d(15.5, 0.5);
        public static Translation2d kRedRightPass = new Translation2d(15.5, 7.5);
        
        public static Translation2d BlueHub = new Translation2d(4.85, 4.0);
        public static Translation2d RedHub = new Translation2d(11.89, 4.035);

        public static double kPassRPM = 250; //3000;
    public static int kPassHoodDeg = 45; //35;
    }

    public static final class HoodConstants {
        // CAN ID for the hood Talon (tunable for your robot wiring)
        public static final int kHoodMotorId = 40;

        // Second calibration setpoint: motor rotations corresponding to the
        // hood angle defined in kHoodAngleB. Tune kHoodSetpointB through testing.
        // A named retracted/home angle constant (degrees)
        public static final double kRetractedAngle = 70.0;

        // Calibration point A: starting/home position
        // Hood angle A (degrees) corresponds to motor rotations A
        public static final double kHoodAngleA = kRetractedAngle;   // degrees (motor position 0)
        public static final double kHoodSetpointA = 0.0; // rotations

        // Calibration point B: max (lower) hood position
        // Hood angle B (degrees) corresponds to motor rotations B
        public static final double kHoodAngleB = 25.0;   // degrees
        public static final double kHoodSetpointB = 5.0; // rotations
        
        // Hood motor current limit (amps)
        public static final int kHoodSupplyCurrentLimit = 20;

        // Position PID gains for hood (slot 0)
        public static final double kHoodKG = 0.0; // feedforward
        public static final double kHoodKP = 2.25; // proportional
        public static final double kHoodKI = 0.01; // integral
        public static final double kHoodKD = 0.05; // derivative
    }

    public static final class EmergencyOperatorControls{
        public static final int OperatorAbort = 1;
        public static final int ReverseIntake = 2;
        public static final int ErrorCorrectionClose = 3;
        public static final int WarmupShooter = 3;
        public static final int ErrorCorrectionFar = 4;
        public static final int VisionOverride = 5;
    }
    public static final class StandardOperatorControls{
        public static final int ToggleIntake = 1;
        public static final int IntakeFuel = 2;
        public static final int ShootFuel = 3;
        public static final int Left = 4;
        public static final int Right = 6;
        /*public static final int Zone4 = 4;
        public static final int Zone5 = 5;
        public static final int Zone6 = 6;*/
        public static final int HoodLow = 7;
        public static final int HoodMid = 8;
        public static final int HoodHigh = 9;
    }
}
    

