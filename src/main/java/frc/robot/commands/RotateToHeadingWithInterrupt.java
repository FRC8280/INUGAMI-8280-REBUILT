package frc.robot.commands;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class RotateToHeadingWithInterrupt extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final SwerveRequest.FieldCentric request =
      new SwerveRequest.FieldCentric().withDeadband(0).withRotationalDeadband(0);

  private final PIDController thetaPid;

  private final Supplier<Rotation2d> currentHeading;
  private final Supplier<Rotation2d> targetHeading;

  private final DoubleSupplier vxMetersPerSec;
  private final DoubleSupplier vyMetersPerSec;

  private final double maxOmegaRadPerSec;

  // If this returns true, we end immediately (joystick moved, etc.)
  private final BooleanSupplier shouldInterrupt;

  // Choose whether you want it to end when at setpoint
  private final boolean finishWhenAtSetpoint;

  public RotateToHeadingWithInterrupt(
      CommandSwerveDrivetrain drivetrain,
      Supplier<Rotation2d> currentHeading,
      Supplier<Rotation2d> targetHeading,
      DoubleSupplier vxMetersPerSec,
      DoubleSupplier vyMetersPerSec,
      double kP, double kI, double kD,
      double maxOmegaRadPerSec,
      BooleanSupplier shouldInterrupt,
      boolean finishWhenAtSetpoint) {

    this.drivetrain = drivetrain;
    this.currentHeading = currentHeading;
    this.targetHeading = targetHeading;
    this.vxMetersPerSec = vxMetersPerSec;
    this.vyMetersPerSec = vyMetersPerSec;
    this.maxOmegaRadPerSec = maxOmegaRadPerSec;
    this.shouldInterrupt = shouldInterrupt;
    this.finishWhenAtSetpoint = finishWhenAtSetpoint;

    thetaPid = new PIDController(kP, kI, kD);
    thetaPid.enableContinuousInput(-Math.PI, Math.PI);
    thetaPid.setTolerance(Math.toRadians(2.0), Math.toRadians(10.0));

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    System.out.printf("*********************Original***  in heading correction: %.2f degrees%n", targetHeading.get().getDegrees());
    System.out.printf("*********************Original***  in heading correction: %.2f degrees%n", targetHeading.get().getDegrees());
    System.out.printf("*********************Original***  in heading correction: %.2f degrees%n", targetHeading.get().getDegrees());
    System.out.printf("*********************Original***  in heading correction: %.2f degrees%n", targetHeading.get().getDegrees());
    System.out.printf("*********************Original***  in heading correction: %.2f degrees%n", targetHeading.get().getDegrees());
    System.out.printf("*********************Original***  in heading correction: %.2f degrees%n", targetHeading.get().getDegrees());
    System.out.printf("*********************Original***  in heading correction: %.2f degrees%n", targetHeading.get().getDegrees());

    thetaPid.reset();
  }

  @Override
  public void execute() {
    double currentRad = currentHeading.get().getRadians();
    double targetRad = targetHeading.get().getRadians();

    System.out.printf("Updated heading correction: %.2f degrees%n", targetHeading.get().getDegrees());


    double omega = thetaPid.calculate(currentRad, targetRad);
    omega = MathUtil.clamp(omega, -maxOmegaRadPerSec, maxOmegaRadPerSec);

    drivetrain.setControl(
        request
            .withVelocityX(vxMetersPerSec.getAsDouble())
            .withVelocityY(vyMetersPerSec.getAsDouble())
            .withRotationalRate(omega));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  @Override
  public boolean isFinished() {
    if (shouldInterrupt.getAsBoolean()) return true;
    if (finishWhenAtSetpoint && thetaPid.atSetpoint()) return true;
    return false;
  }
}