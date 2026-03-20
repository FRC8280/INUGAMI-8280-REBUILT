package frc.robot;

import java.util.List;
import java.util.Set;

import edu.wpi.first.math.geometry.Translation2d;

public class Constants {

    public static final class LEDConstants {
        public static final int kFrontCandleID = 50; // CAN bus ID for the CANdle
        public static final int kTimerCandleID = 51; // CAN bus ID for the timer candle (if used)

    }


    public static final boolean kVerboseDashboard = true;
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
        public static final double kPivotDeployed = -21.796;
        public static final double kPivotAgitate = -7.683;

        public static final int kRollerIntakeMotorId = 12;
        public static final int kSpeed = 3000;
        public static final int kRollerCurrentLimit =40;

        public static final int kAbsEncoderId = 45;
        public static final double kCanCoderZero = 0.128; // Set this to the absolute encoder reading when the intake is in the stowed position
    }

    public static final class IndexerConstants{
        public static final int kIndexerMotorId = 11;
        public static final int kIndexerCurrentLimit = 40;
        public static final int kSpeed = 4500;//1000;

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
        public static final double kP = 0.11;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0.1;
        public static final double kV = 0.12;

        //Accelerator PID values (if using)
        public static final double kAccelP = .2;
        public static final double kAccelI = 0;
        public static final double kAccelD = 0;
        public static final double kAccelS = 0.1;
        public static final double kAccelV = 0.12;

        public static final int kShooterMotorId = 30;
        public static final int kShooterCurrentLimit = 40;
        public static final int kFollowerMotorId = 31;

        public static final int kAccelerateCurrentLimit = 30;
        public static final int kAccelerateMotorId = 16;
        
        public static final int kServoHubId = 10;

        public static final double kShooterTargetVelocity = 240; // RPM
        public static final double kDefaultShotAngleDeg = 60.0; // Default shot angle in degrees
        public static final double kHoodMinDeg = 0.0; // Minimum hood angle in degrees
        public static final double kHoodMaxDeg = 15.0; // Maximum hood angle in degrees

        public static final Set<Integer> BLUE_TAGS = Set.of(25,26,21,24,18,27);
        public static final Set<Integer> RED_TAGS = Set.of(9,10,11,12,8,5);

        public static final double kShooterDefaultAngle = 60.0;
        public static final double kWarmupSeconds = 2.0;

        public static final List<Translation2d> PASSING_POSES = List.of(
            new Translation2d(2, 7), // 0   //blue left
            new Translation2d(2, 1), // 1   //blue right

            new Translation2d(14, 7), // 2  //red left
            new Translation2d(14, 1)  // 3  //red right
        );
        public static Translation2d BlueHub = new Translation2d(4.85, 4.0);
        public static Translation2d RedHub = new Translation2d(11.89, 4.035);

        

    }

    public static final class EmergencyOperatorControls{
        public static final int OperatorAbort = 1;
        public static final int ReverseIntake = 2;
        public static final int ErrorCorrectionClose = 3;
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
        public static final int Zone6 = 6;
        public static final int Zone1 = 7;
        public static final int Zone2 = 8;
        public static final int Zone3 = 9;*/
    }
}
    

