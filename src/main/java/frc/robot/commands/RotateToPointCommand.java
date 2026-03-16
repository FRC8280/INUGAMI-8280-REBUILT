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

    }
}