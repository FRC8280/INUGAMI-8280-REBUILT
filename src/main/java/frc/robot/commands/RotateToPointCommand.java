package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RotateToPointCommand extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<Translation2d> targetSupplier;
    public final RobotContainer m_RobotContainer;
     private final BooleanSupplier m_AbortSupplier;

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();

    private final PIDController headingPID = new PIDController(6.0, 0.0, 0.25);

    public RotateToPointCommand(
            CommandSwerveDrivetrain drivetrain,
            Supplier<Pose2d> poseSupplier,
            Supplier<Translation2d> targetSupplier,
            RobotContainer robotContainer,
            BooleanSupplier abortSupplier) {

        this.drivetrain = drivetrain;
        this.poseSupplier = poseSupplier;
        this.targetSupplier = targetSupplier;
        this.m_RobotContainer = robotContainer;
        this.m_AbortSupplier = abortSupplier;

        headingPID.enableContinuousInput(-Math.PI, Math.PI);
        headingPID.setTolerance(Math.toRadians(2)); // ~2 degrees

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        headingPID.reset();
    }

    @Override
    public void execute() {

        Pose2d pose = poseSupplier.get();
        Translation2d target = targetSupplier.get();

        double dx = target.getX() - pose.getX();
        double dy = target.getY() - pose.getY();

        double targetAngle = Math.atan2(dy, dx);
        double currentAngle = pose.getRotation().getRadians();

        // compute normalized angle error in [-pi, pi]
        double angleError = Math.atan2(Math.sin(targetAngle - currentAngle), Math.cos(targetAngle - currentAngle));

        // Choose threshold depending on alliance (30° normally, 145° when on Red)
        double thresholdDeg = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) ? 140.0 : 30.0;
        // if the absolute angle to target is greater than the threshold, apply a 7-degree bias
        if (Math.abs(angleError) > Math.toRadians(thresholdDeg)) {
            double bias = Math.toRadians(15.5);//7.0);
            if (angleError < 0) {
                targetAngle -= bias; // compensate more negative
            } else {
                targetAngle += bias; // compensate more positive
            }
            // recompute angleError after bias (optional)
            angleError = Math.atan2(Math.sin(targetAngle - currentAngle), Math.cos(targetAngle - currentAngle));
        }

        double omega = headingPID.calculate(currentAngle, targetAngle);

        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(omega)
        );
    }

    @Override
    public boolean isFinished() {
        return headingPID.atSetpoint() || m_AbortSupplier.getAsBoolean();
    }
    

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(
            driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
        m_RobotContainer.HitBrakes();
        m_RobotContainer.fIsAutoAiming = true;

    }
}