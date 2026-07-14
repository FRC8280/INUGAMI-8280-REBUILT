package frc.robot.commands.Swerve;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToTargetDropIn extends Command {

    // INPUT ROBOT WIDTH IN INCHES
    private static double ROBOT_WIDTH = 37;

    // ROBOT SPEED PERCENTAGE, 0-1
    private double SPEED_LIMIT = 1;

    // FROM BLUE ALLIANCE PERSPECTIVE, THE DIRECTION THE ROBOT SHOULD FACE
    private double ROBOT_ROTATION_DEGREES = 0;

    private double PID_X_AND_Y_P = 4;
    private double PID_ROTATION_P = 5.5;


    // public Pose2d getPose2d() {
    //     return this.getState().Pose;
    // }
    // driverJoystick.rightBumper().whileTrue(new DriveToTargetDropIn(drivetrain, () -> drivetrain.getPose2d(), () -> DriverStation.getAlliance().get(), "Right"));
    // driverJoystick.leftBumper().whileTrue(new DriveToTargetDropIn(drivetrain, () -> drivetrain.getPose2d(), () -> DriverStation.getAlliance().get(), "Left"));
   
    private CommandSwerveDrivetrain drivetrain;
    private SwerveRequest swerveRequest;
    private Pose2d currentPose, targetPose;
    private Supplier<Pose2d> currentPoseSupplier;

    // CONSTANTS

    public static double inchesToMeters(double inches) {
        return inches / 39.37;
    }

    private static double MAX_ANGULAR_VELOCITY_RADIANS = 3 * Math.PI;
    private static final double MAX_ANGULAR_ACCELERATION_RPSS = Math.PI * 6; // Radians Per Second Squared
    
    private static final double ROBOT_SIZE_X_OFFSET_METERS = inchesToMeters(ROBOT_WIDTH * 0.5);
    
    private Pose2d RED_ZONE_RIGHT_POSE = new Pose2d(inchesToMeters(500) + ROBOT_SIZE_X_OFFSET_METERS, 2.5, Rotation2d.fromDegrees(ROBOT_ROTATION_DEGREES));
    private Pose2d RED_ZONE_LEFT_POSE = new Pose2d(inchesToMeters(500) + ROBOT_SIZE_X_OFFSET_METERS, 5.5, Rotation2d.fromDegrees(ROBOT_ROTATION_DEGREES));

    private Pose2d BLUE_ZONE_RIGHT_POSE = new Pose2d(inchesToMeters(150) - ROBOT_SIZE_X_OFFSET_METERS, 5.5, Rotation2d.fromDegrees(ROBOT_ROTATION_DEGREES + 180));
    private Pose2d BLUE_ZONE_LEFT_POSE = new Pose2d(inchesToMeters(150) - ROBOT_SIZE_X_OFFSET_METERS, 2.5, Rotation2d.fromDegrees(ROBOT_ROTATION_DEGREES + 180));

    private Pose2d RED_ZONE_RIGHT_TRENCH_POSE = new Pose2d(inchesToMeters(500) + ROBOT_SIZE_X_OFFSET_METERS, 7.481, Rotation2d.fromDegrees(ROBOT_ROTATION_DEGREES));
    private Pose2d RED_ZONE_LEFT_TRENCH_POSE = new Pose2d(inchesToMeters(500) + ROBOT_SIZE_X_OFFSET_METERS, 0.576, Rotation2d.fromDegrees(ROBOT_ROTATION_DEGREES));

    private Pose2d BLUE_ZONE_RIGHT_TRENCH_POSE = new Pose2d(inchesToMeters(150) - ROBOT_SIZE_X_OFFSET_METERS, 0.576, Rotation2d.fromDegrees(ROBOT_ROTATION_DEGREES + 180));
    private Pose2d BLUE_ZONE_LEFT_TRENCH_POSE = new Pose2d(inchesToMeters(150) - ROBOT_SIZE_X_OFFSET_METERS, 7.481, Rotation2d.fromDegrees(ROBOT_ROTATION_DEGREES + 180));


    private final HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
        new PIDController(PID_X_AND_Y_P, 0, 0), 
        new PIDController(PID_X_AND_Y_P, 0, 0), 
        new ProfiledPIDController(PID_ROTATION_P, 0, 0, new TrapezoidProfile.Constraints(MAX_ANGULAR_VELOCITY_RADIANS, MAX_ANGULAR_ACCELERATION_RPSS))
    );

    // Non Constant
    private Supplier<Alliance> allianceColorSupplier;
    private String side;


    private SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /**
     * 
     * @param drivetrain
     * @param currentPoseSupplier
     * @param allianceColorSupplier
     * @param side
     */
    public DriveToTargetDropIn(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> currentPoseSupplier, Supplier<Alliance> allianceColorSupplier, String side) {
        this.drivetrain = drivetrain;
        this.currentPoseSupplier = currentPoseSupplier;
        this.allianceColorSupplier = allianceColorSupplier;
        this.side = side;
        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize(){
        holonomicDriveController.setTolerance(new Pose2d(inchesToMeters(0.1), inchesToMeters(0.1), Rotation2d.fromDegrees(2)));
        if (allianceColorSupplier.get() == Alliance.Blue) {
            if (side.equals("RightBump")) {
                targetPose = RED_ZONE_RIGHT_POSE;
            } else if (side.equals("RightTrench")) {
                targetPose = RED_ZONE_LEFT_TRENCH_POSE;
            } else if (side.equals("LeftBump")) {
                targetPose = RED_ZONE_LEFT_POSE;
            } else if (side.equals("LeftTrench")) {
                targetPose = RED_ZONE_RIGHT_TRENCH_POSE;
            }
        } else {
            if (side.equals("RightBump")) {
                targetPose = BLUE_ZONE_LEFT_POSE;
            } else if (side.equals("RightTrench")) {
                targetPose = BLUE_ZONE_RIGHT_TRENCH_POSE;
            } else if (side.equals("LeftBump")) {
                targetPose = BLUE_ZONE_RIGHT_POSE;
            } else if (side.equals("LeftTrench")) {
                targetPose = BLUE_ZONE_LEFT_TRENCH_POSE;
            }
        }
    }

    @Override
    public void execute() {
        currentPose = currentPoseSupplier.get();
        ChassisSpeeds speeds = holonomicDriveController.calculate(
            currentPose, 
            targetPose, 
            0, 
            targetPose.getRotation()
        );

        speeds.vxMetersPerSecond *= SPEED_LIMIT;
        speeds.vyMetersPerSecond *= SPEED_LIMIT;
        
        swerveRequest = robotCentric
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withRotationalRate(speeds.omegaRadiansPerSecond);

        drivetrain.setControl(swerveRequest);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(robotCentric.withVelocityX(0).withVelocityY(0));
    }

    @Override
    public boolean isFinished() {
        return holonomicDriveController.atReference();
    }
}
