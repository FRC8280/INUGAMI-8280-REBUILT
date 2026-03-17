// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;

import com.pathplanner.lib.auto.NamedCommands;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import frc.robot.commands.RotateToPointCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

    enum AlignmentState {
        IDLE,
        ALIGNING
    }

    private AlignmentState m_alignmentState = AlignmentState.IDLE;

    enum ShootingState {
        IDLE,
        WARMUP,
        ALIGNMENT,
        FIRING
    }

    enum PassingZone {
        NOT_PASSING,
        Zone1,
        Zone2,
        Zone3,
        Zone4,
        Zone5,
        Zone6
    }

    private PassingZone passingZone = PassingZone.NOT_PASSING;
    private ShootingState m_shootingState = ShootingState.IDLE;

    private PowerDistribution powerDistributionSystem = new PowerDistribution();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    // Supplier defined here
    private final Supplier<Pose2d> drivePoseSupplier = () -> drivetrain.getState().Pose;

    private final LEDSubsystem ledSystem = new LEDSubsystem();
    private final Supplier<LEDSubsystem> ledSupplier = () -> ledSystem;

    private final Intake m_Intake = new Intake();
    private final Indexer m_Indexer = new Indexer();
    private final Shooter m_Shooter = new Shooter(drivePoseSupplier, ledSupplier);

    private final Timer m_intakeCycleTimer = new Timer();
    private final Timer m_intakeDeployTimer = new Timer();
    private static final double kIntakeCycleSeconds = 1.0;
    private static final double kIntakeDeployDuration = 0.25;
    private boolean m_intakeCycleRunning = false;

    private double driveScaler = 0.70;
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final Joystick operatorEmergency = new Joystick(1);
    private final Joystick operatorStandard = new Joystick(2);

    private final double deadzone = 0.10; // Adjust this value as needed

    // Example axis suppliers (adjust for your controller)
    private final DoubleSupplier xAxis = () -> -driver.getLeftY(); // forward/back
    private final DoubleSupplier yAxis = () -> -driver.getLeftX(); // strafe
    private final DoubleSupplier rAxis = () -> -driver.getRightX(); // manual rotate

    private final Timer m_warmupTimer = new Timer();
    
    private Command autoAimCommand = null;

    private boolean driverOverride() {
        return Math.abs(xAxis.getAsDouble()) > deadzone
                || Math.abs(yAxis.getAsDouble()) > deadzone
                || Math.abs(rAxis.getAsDouble()) > deadzone;
    }

    private double limelight_aim_proportional() {
        double kP = .04375; // .035;
        double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;
        targetingAngularVelocity *= MaxAngularRate;
        targetingAngularVelocity *= -1.0;
        return targetingAngularVelocity;
    }

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        NamedCommands.registerCommand("Stop Shooter", new InstantCommand(() -> CeaseFire()));
        NamedCommands.registerCommand("Shoot", new InstantCommand(() -> StartFiringSequence()));

        NamedCommands.registerCommand("Deploy Intake", new InstantCommand(() -> m_Intake.deployIntake()));
        NamedCommands.registerCommand("Stow Intake", new InstantCommand(() -> m_Intake.stowIntake()));

        NamedCommands.registerCommand("Aim", new InstantCommand(() -> this.ExecuteAimCommand(m_Shooter.GetAllianceHub())));

        NamedCommands.registerCommand("Start Intake", new InstantCommand(() -> m_Intake.startIntake()));
        NamedCommands.registerCommand("Stop Intake", new InstantCommand(() -> m_Intake.stopIntake()));

        // DriverStation.silenceJoystickConnectionWarning(true);
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

        DogLog.setOptions(new DogLogOptions().withCaptureDs(true));

    }

    public void periodic() {
        SmartDashboard.putNumber("PowerSystem/Voltage", powerDistributionSystem.getVoltage());
        SmartDashboard.putNumber("PowerSystem/Current", powerDistributionSystem.getTotalCurrent());
    }

    public void SetShootingSTate(ShootingState state) {
        m_shootingState = state;
    }

    public void StartFiringSequence() {
        m_warmupTimer.reset();
        m_warmupTimer.start();

        m_shootingState = ShootingState.WARMUP;
        m_Shooter.WarmupShooter();
    }

    public void OpenFire() {
        m_shootingState = ShootingState.FIRING;
        m_Indexer.startIndexer();
        m_warmupTimer.stop();
    }

    public boolean ReadyToFire() {

        double elapsed = m_warmupTimer.get();
        if (elapsed >= Constants.kWarmupSeconds) // Timeout of it's taking too long
            return true;

        // return m_Shooter.IsReadyToFire();
        return m_Shooter.IsReadyToFire() && (autoAimCommand == null || !autoAimCommand.isScheduled());
    }

    boolean fIntakeUp = false;

    public void toggleIntake() {
        if (fIntakeUp) {
            m_Intake.deployIntake();
            fIntakeUp = false;
        } else {
            m_Intake.stowIntake();
            fIntakeUp = true;
        }
    }

    public void CeasePassing(PassingZone buttonID){

        if(buttonID != passingZone  )
            return;

        passingZone = PassingZone.NOT_PASSING;   
        CeaseFire(); 
    }
    public void CeaseFire() {

        //todo: verify this logic
        if(passingZone != PassingZone.NOT_PASSING)
            return;

        // Todo stop any auto aiming system
        m_intakeCycleTimer.stop();
        m_intakeDeployTimer.stop();

        m_shootingState = ShootingState.IDLE;
        TerminateAimCommand();
        m_Shooter.CeaseFire();
        m_Indexer.stopIndexer();
        m_warmupTimer.stop();
        m_alignmentState = AlignmentState.IDLE;
        m_Intake.deployIntake();
    }

    private Command ExecuteAimCommand(Translation2d target) {
        autoAimCommand = new RotateToPointCommand(
                drivetrain,
                drivePoseSupplier, // robot pose supplier
                () -> target, // target point on field
                this,
                () -> driverOverride());

        return autoAimCommand;
    }

    public void HitBrakes() {
        drivetrain.applyRequest(() -> brake);
    }

    private void TerminateAimCommand() {

        if (autoAimCommand != null && autoAimCommand.isScheduled()) {
            autoAimCommand.cancel();
            autoAimCommand = null;
        }
    }

    double speed = 1.0;
    private void configureBindings() {

        /*if(m_Intake.isIntakeRunning())
            speed = 0.75; 
        else
            speed = 1;*/

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * MaxSpeed *driveScaler ) // Drive forward with
                                                                                                 // negative Y (forward)
                        .withVelocityY(-driver.getLeftX() * MaxSpeed * driveScaler  ) // Drive left with negative X (left)
                        .withRotationalRate(-driver.getRightX() * MaxAngularRate  ) // Drive counterclockwise with
                                                                               
                        // negative X (left)
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        new Trigger(() -> Math.abs(driver.getLeftY()) < deadzone
                && Math.abs(driver.getLeftX()) < deadzone
                && Math.abs(driver.getRightX()) < deadzone
                && m_alignmentState != AlignmentState.ALIGNING)
                .whileTrue(drivetrain.applyRequest(() -> brake));

        driver.b().whileTrue(drivetrain
                .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

        // Reset the field-centric heading on left bumper press.
        driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // emergency servo reset
        driver.povDown().onTrue(new InstantCommand(() -> m_Shooter.m_Hood.retractServo()));

        // Firing logic
        driver.rightTrigger().whileTrue(new InstantCommand(() -> StartFiringSequence())
                .alongWith(ExecuteAimCommand(m_Shooter.GetAllianceHub())));
        driver.rightTrigger().onFalse(new InstantCommand(() -> CeaseFire()));

        new Trigger(() -> ReadyToFire())
                .onTrue(new InstantCommand(() -> {
                    OpenFire();
                })); // activate the indexer once the shooter is warmed up and ready to fire.

        // intake logic
        driver.x().onTrue(new InstantCommand(() -> toggleIntake()));
        driver.a().whileTrue((new InstantCommand(() -> m_Intake.startIntake())));
        driver.a().onFalse((new InstantCommand(() -> m_Intake.stopIntake())));

        // Aiming logic
        driver.y().onTrue(ExecuteAimCommand(m_Shooter.GetAllianceHub()));

        // ***********************************************operator
        // controls********************************************************
        JoystickButton toggleButton = new JoystickButton(operatorStandard,
                Constants.StandardOperatorControls.ToggleIntake);
        toggleButton.onTrue(new InstantCommand(() -> toggleIntake()));

        JoystickButton intakeFuelButton = new JoystickButton(operatorStandard,
                Constants.StandardOperatorControls.IntakeFuel);
        intakeFuelButton.onTrue(new InstantCommand(() -> m_Intake.startIntake()));
        intakeFuelButton.onFalse(new InstantCommand(() -> m_Intake.stopIntake()));

        JoystickButton shootFuelButton = new JoystickButton(operatorStandard,
                Constants.StandardOperatorControls.ShootFuel);
        shootFuelButton.whileTrue(new InstantCommand(() -> StartFiringSequence())
                .alongWith(ExecuteAimCommand(m_Shooter.GetAllianceHub())));
        shootFuelButton.onFalse(new InstantCommand(() -> CeaseFire()));

        // While the shooter is firing, periodically bring the intake up to feed,
        // then stow it after a short deploy duration. Stops when shooter stops firing.
        new Trigger(() -> this.ReadyToFire()) // m_Shooter.IsReadyToFire())
                .onTrue(new RunCommand(() -> {
                    if (!m_intakeCycleRunning) {
                        m_intakeCycleTimer.reset();
                        m_intakeDeployTimer.reset();
                        m_intakeCycleTimer.start();
                        m_intakeDeployTimer.stop();
                        m_intakeCycleRunning = true;
                    }
                }))

                .onFalse(new InstantCommand(() -> {
                    m_intakeCycleTimer.stop();
                    m_intakeDeployTimer.stop();
                    m_intakeCycleRunning = false;
                    m_Intake.stopIntake();
                    m_Intake.deployIntake();
                }));

        new Trigger(() -> m_shootingState == ShootingState.FIRING
                && m_intakeCycleTimer.get() >= kIntakeCycleSeconds
                && !m_intakeDeployTimer.isRunning())
                .onTrue(new InstantCommand(() -> {
                    m_Intake.agitateIntake();
                    m_Intake.startIntake();
                    m_intakeDeployTimer.reset();
                    m_intakeDeployTimer.start();
                    m_intakeCycleTimer.reset();
                }));

        // When in FIRING and the deploy timer has expired, stop intake and stow it.
        new Trigger(() -> m_shootingState == ShootingState.FIRING
                && m_intakeDeployTimer.isRunning()
                && m_intakeDeployTimer.get() >= kIntakeDeployDuration)
                .onTrue(new InstantCommand(() -> {
                    m_Intake.stopIntake();
                    m_Intake.deployIntake();
                    m_intakeDeployTimer.stop();
                }));

        JoystickButton abortButton = new JoystickButton(operatorEmergency,
                Constants.EmergencyOperatorControls.OperatorAbort);
        abortButton.onTrue(new InstantCommand(() -> CeaseFire())
                .alongWith(new InstantCommand(() -> driverOverride()))
                .alongWith(new InstantCommand(() -> {
                    m_intakeDeployTimer.stop();
                    m_intakeCycleTimer.stop();
                    m_intakeCycleRunning = false;
                }))
                .alongWith(new InstantCommand(() -> m_Intake.deployIntake()))
                .alongWith(new InstantCommand(() -> m_Intake.stopIntake())));

        JoystickButton intakeReverseButton = new JoystickButton(operatorEmergency,
                Constants.EmergencyOperatorControls.ReverseIntake);
        intakeReverseButton.onTrue(new InstantCommand(() -> m_Intake.reverseIntake()));
        intakeReverseButton.onFalse(new InstantCommand(() -> m_Intake.stopIntake()));

        //Passing Controls
        JoystickButton passZone1 = new JoystickButton(operatorStandard, Constants.StandardOperatorControls.Zone1);
        passZone1.whileTrue(new InstantCommand(() -> passingZone = PassingZone.Zone1)
                .alongWith(ExecuteAimCommand(m_Shooter.getPassingPose(0)))
                .alongWith(new InstantCommand(()->StartFiringSequence())));
        passZone1.onFalse(new InstantCommand(() -> CeasePassing(PassingZone.Zone1)));
    
        JoystickButton passZone2 = new JoystickButton(operatorStandard, Constants.StandardOperatorControls.Zone2);
        passZone2.whileTrue(new InstantCommand(() -> passingZone = PassingZone.Zone2)
                .alongWith(ExecuteAimCommand(m_Shooter.getPassingPose(1)))
                .alongWith(new InstantCommand(()->StartFiringSequence())));
        passZone2.onFalse(new InstantCommand(() -> CeasePassing(PassingZone.Zone2)));

        JoystickButton passZone3 = new JoystickButton(operatorStandard, Constants.StandardOperatorControls.Zone3);
        passZone3.whileTrue(new InstantCommand(() -> passingZone = PassingZone.Zone3)
                .alongWith(ExecuteAimCommand(m_Shooter.getPassingPose(2)))
                .alongWith(new InstantCommand(()->StartFiringSequence())));
        passZone3.onFalse(new InstantCommand(() -> CeasePassing(PassingZone.Zone3)));

        JoystickButton passZone4 = new JoystickButton(operatorStandard, Constants.StandardOperatorControls.Zone4);
        passZone4.whileTrue(new InstantCommand(() -> passingZone = PassingZone.Zone4)
                .alongWith(ExecuteAimCommand(m_Shooter.getPassingPose(3)))
                .alongWith(new InstantCommand(()->StartFiringSequence())));
        passZone4.onFalse(new InstantCommand(() -> CeasePassing(PassingZone.Zone4)));   

        JoystickButton passZone5 = new JoystickButton(operatorStandard, Constants.StandardOperatorControls.Zone5);
        passZone5.whileTrue(new InstantCommand(() -> passingZone = PassingZone.Zone5)
                .alongWith(ExecuteAimCommand(m_Shooter.getPassingPose(4)))
                .alongWith(new InstantCommand(()->StartFiringSequence())));
        passZone5.onFalse(new InstantCommand(() -> CeasePassing(PassingZone.Zone5)));   

        JoystickButton passZone6 = new JoystickButton(operatorStandard, Constants.StandardOperatorControls.Zone6);
        passZone6.whileTrue(new InstantCommand(() -> passingZone = PassingZone.Zone6)
                .alongWith(ExecuteAimCommand(m_Shooter.getPassingPose(5)))
                .alongWith(new InstantCommand(()->StartFiringSequence())));
        passZone6.onFalse(new InstantCommand(() -> CeasePassing(PassingZone.Zone6)));

        // if(m_shootingState == ShootingState.FIRING || m_shootingState ==
        // ShootingState.WARMUP)
        /*
         * driver.leftTrigger().onTrue(
         * drivetrain.applyRequest(() ->
         * drive.withVelocityX(-driver.getLeftY() * MaxSpeed)
         * .withVelocityY(-driver.getLeftX() * MaxSpeed)
         * .withRotationalRate(limelight_aim_proportional())));
         * 
         * driver.leftTrigger().onFalse(
         * drivetrain.applyRequest(() ->
         * drive.withVelocityX(-driver.getLeftY() * MaxSpeed)
         * .withVelocityY(-driver.getLeftX() * MaxSpeed)
         * .withRotationalRate(-driver.getRightX() * MaxAngularRate)));
         */

        // Call TestPreShotMotor(true) while left bumper is pressed, and
        // TestPreShotMotor(false) when released
        // driver.leftBumper().onTrue(new InstantCommand(() ->
        // m_Shooter.testPreShotMotor(true)));
        // driver.leftBumper().onFalse(new InstantCommand(() ->
        // m_Shooter.testPreShotMotor(false)));

        // Call set velocity on shooter when a button is held down.
        // driver.a().whileTrue(new InstantCommand(() ->
        // m_Shooter.setShooterVelocity(50*60)));
        // driver.a().onFalse(new InstantCommand(() ->
        // m_Shooter.setShooterVelocity(0)));

        // Activate shooter + indexer while right trigger is held, stop both when
        // released.
        /*
         * driver.rightTrigger().whileTrue(
         * new InstantCommand(() -> m_Shooter.setShooterVelocity(50*60))
         * .alongWith(new InstantCommand(() -> m_Shooter.testPreShotMotor(true)))
         * );
         * driver.rightTrigger().onFalse(
         * new InstantCommand(() -> m_Shooter.setShooterVelocity(0))
         * .alongWith(new InstantCommand(() -> m_Shooter.testPreShotMotor(false)))
         * );
         */

        // Activate intake while Y is pressed, stop when released
        /*
         * driver.y().onTrue(
         * new InstantCommand(() -> m_Intake.deployIntake()));
         * driver.x().onFalse(
         * new InstantCommand(() -> m_Intake.stowIntake()));
         */

        // Sys ID code
        /*
         * driver.povUp().whileTrue(drivetrain.applyRequest(() ->
         * forwardStraight.withVelocityX(0.5).withVelocityY(0))
         * );
         * driver.povDown().whileTrue(drivetrain.applyRequest(() ->
         * forwardStraight.withVelocityX(-0.5).withVelocityY(0))
         * );
         * 
         * // Run SysId routines when holding back/start and X/Y.
         * // Note that each routine should be run exactly once in a single log.
         * driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.
         * kForward));
         * driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.
         * kReverse));
         * driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(
         * Direction.kForward));
         * driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(
         * Direction.kReverse));
         */

        // Call TestPreShotMotor(true) while A is pressed, and TestPreShotMotor(false)
        // when released
        /*
         * driver.a().onTrue(new InstantCommand(() ->
         * m_Shooter.TestPreShotMotor(true)));
         * driver.a().onFalse(new InstantCommand(() ->
         * m_Shooter.TestPreShotMotor(false)));
         */

        /*
         * operator.a().onTrue(
         * new InstantCommand(() -> m_Intake.startIntake())
         * .alongWith(new InstantCommand(() -> m_Indexer.StartIndexer()))
         * );
         * 
         * operator.b().onTrue(
         * new InstantCommand(() -> m_Intake.reverseIntake())
         * .alongWith(new InstantCommand(() -> m_Indexer.ReverseIndexer()))
         * );
         * 
         * operator.a().onFalse(
         * new InstantCommand(() -> m_Intake.stopIntake())
         * .alongWith(new InstantCommand(() -> m_Indexer.StopIndexer()))
         * );
         * operator.b().onFalse(
         * new InstantCommand(() -> m_Intake.stopIntake())
         * .alongWith(new InstantCommand(() -> m_Indexer.StopIndexer()))
         * );
         * 
         * 
         * operator.leftTrigger().onTrue(new InstantCommand(() ->
         * m_Intake.deployIntake()));
         * operator.rightTrigger().onTrue(new InstantCommand(() ->
         * m_Intake.stowIntake()));
         */

        // drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

}
