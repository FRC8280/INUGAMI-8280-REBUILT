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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;

import com.pathplanner.lib.auto.NamedCommands;

//import dev.doglog.DogLog;
//import dev.doglog.DogLogOptions;
import frc.robot.commands.RotateToPointCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

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
        Zone_Left,
        Zone_Right
    }


    private PassingZone passingZone = PassingZone.NOT_PASSING;
    private ShootingState m_shootingState = ShootingState.IDLE;

    //private PowerDistribution powerDistributionSystem = new PowerDistribution();
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    // Supplier defined here
    private final Supplier<Pose2d> drivePoseSupplier = () -> drivetrain.getState().Pose;

    public final LEDSubsystem ledSystem = new LEDSubsystem();
    private final Supplier<LEDSubsystem> ledSupplier = () -> ledSystem;

    private final Intake m_Intake = new Intake();
    private final Indexer m_Indexer = new Indexer();
    private final Shooter m_Shooter = new Shooter(drivePoseSupplier, ledSupplier);

    private final Timer m_intakeCycleTimer = new Timer();
    private final Timer m_intakeDeployTimer = new Timer();
    private final Timer teleopTimer = new Timer();
    private static final double kIntakeCycleSeconds = 0.5;
    private static final double kIntakeDeployDuration = 0.25;
    private boolean m_intakeCycleRunning = false;

    private double driveScaler = 0.85;
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentricFacingAngle aimDrive = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.10)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final Joystick operatorEmergency = new Joystick(1);
    private final Joystick operatorStandard = new Joystick(2);

    private final double deadzone = 0.10; // Adjust this value as needed

    // Example axis suppliers (adjust for your controller)
    private final DoubleSupplier xAxis = () -> -driver.getLeftY(); // forward/back
    private final DoubleSupplier yAxis = () -> -driver.getLeftX(); // strafe
    private final DoubleSupplier rAxis = () -> -driver.getRightX(); // manual rotate

    private final Timer m_warmupTimer = new Timer();

   

    private boolean driverOverride() {
       /*  return Math.abs(xAxis.getAsDouble()) > deadzone
                || Math.abs(yAxis.getAsDouble()) > deadzone || */
                return Math.abs(rAxis.getAsDouble()) > deadzone;
    }

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        NamedCommands.registerCommand("Stop Shooter", new InstantCommand(() -> CeaseFire()));
        NamedCommands.registerCommand("Shoot", new InstantCommand(() -> StartFiringSequence()));
        NamedCommands.registerCommand("Deploy Intake", new InstantCommand(() -> m_Intake.deployIntake()));
        NamedCommands.registerCommand("Stow Intake", new InstantCommand(() -> m_Intake.stowIntake()));
        NamedCommands.registerCommand("Start Intake", new InstantCommand(() -> m_Intake.startIntake()));
        NamedCommands.registerCommand("Stop Intake", new InstantCommand(() -> m_Intake.stopIntake()));
        NamedCommands.registerCommand("Warmup Shooter", new InstantCommand(() -> m_Shooter.engageShooterIdle()));

        // DriverStation.silenceJoystickConnectionWarning(true);
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Tune these to taste
        aimDrive.HeadingController.setPID(8.0, 0.0, 0.2);
        aimDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

        // DogLog.setOptions(new DogLogOptions().withCaptureDs(true));

    }

    public LEDSubsystem getLEDSystem() {
        return ledSystem;
    }

    public void periodic() {
        /*SmartDashboard.putNumber("PowerSystem/Voltage", powerDistributionSystem.getVoltage());
        SmartDashboard.putNumber("PowerSystem/Current", powerDistributionSystem.getTotalCurrent());*/
        // countdownLED.periodic();
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

    // Todo - this should be inside of the intake class and not in the robot logic
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

    public void CeasePassing(PassingZone buttonID) {

        if (buttonID != passingZone)
            return;

        passingZone = PassingZone.NOT_PASSING;
        // Disable passing mode on the shooter so periodic() returns to normal setpoints
        m_Shooter.disablePassingMode();
        CeaseFire();
    }

    /**
     * Start the passing sequence: enable shooter passing-mode setpoints and
     * begin the normal warmup/firing sequence.
     */
    public void StartPassingSequence(PassingZone zone) {
        passingZone = zone;
        // Use constants for passing RPM/hood angle (from ShooterConstants)
        m_Shooter.enablePassingMode(Constants.ShooterConstants.kPassRPM, Constants.ShooterConstants.kPassHoodDeg);
        // Begin normal warmup/firing sequence (this will start the warmup timer and
        // set the shooter velocity using the passing setpoint already applied in
        // Shooter.periodic()).
        StartFiringSequence();
    }

    public void CeaseFire() {

        if (passingZone != PassingZone.NOT_PASSING)
            return;

        fIsAutoAiming = false;
        
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

    // PID for auto-aim rotational control (radians)
    private final PIDController m_autoAimPID = new PIDController(4.0, 0.0, 0.25);
    {
        m_autoAimPID.enableContinuousInput(-Math.PI, Math.PI);
        m_autoAimPID.setTolerance(Math.toRadians(2.0)); // ~2 degrees
    }

   

    // Supplier defined here
    // PID used for auto-aiming (outputs radians/sec). Tune these gains.
    private final PIDController aimPID = new PIDController(6.0, 0.0, 0.8);

    // Configure aimPID to treat the input as continuous over [0, 2PI).
    {
        aimPID.enableContinuousInput(0.0, 2.0 * Math.PI);
        aimPID.setTolerance(Math.toRadians(2.0));
    }

    public boolean fIsAutoAiming = false;
    private Command autoAimCommand = null;
    private Translation2d lastTarget = m_Shooter.GetAllianceHub();
    
    //Make these member variables to reduce number of math calls
    private double m_desiredAngle = 0.0;
    private double m_currentAngle = 0.0;
    private double m_angleError = 0.0;
    private double m_matchPercent = 0.0;

    private Command ExecuteAimCommand(Translation2d target) {

        lastTarget = target;
        autoAimCommand = new RotateToPointCommand(
                drivetrain,
                drivePoseSupplier, // robot pose supplier
                () -> target, // target point on field
                this,
                () -> driverOverride());

        return autoAimCommand;
    }

    // Normalize radians to [0, 2PI)
    private static double normalizeRadians0To2Pi(double angle) {
        double a = angle % (2.0 * Math.PI);
        if (a < 0)
            a += 2.0 * Math.PI;
        return a;
    }

    // Convert radians to degrees in [0, 360)
    private static double radiansToDegrees360(double radians) {
        double deg = Math.toDegrees(radians) % 360.0;
        if (deg < 0)
            deg += 360.0;
        return deg;
    }

    //Simplify the caluculations by doing this once per cycle. 
    //Allowing other functions to use these values without calculating them. 
    private void CalculateTargetAngles()
    {
        if(!fIsAutoAiming || lastTarget == null)  //if not autoaiming or no target, reset values and return
        {
            m_desiredAngle = 0.0;
            m_currentAngle = 0.0;
            m_angleError = 0.0;
            m_matchPercent = 0.0;
            return;
        }

        var pose = drivetrain.getState().Pose;
        double dx = lastTarget.getX() - pose.getX();
        double dy = lastTarget.getY() - pose.getY();
        // Compute raw angles (radians)
        double desiredRaw = Math.atan2(dy, dx);
        double currentRaw = pose.getRotation().getRadians();

        // Normalize both angles to [0, 2PI) to avoid wraparound issues
        m_desiredAngle = normalizeRadians0To2Pi(desiredRaw);
        m_currentAngle = normalizeRadians0To2Pi(currentRaw);

        // Compute shortest signed angle error in radians (in range [-PI, PI])
        m_angleError = Math.atan2(Math.sin(m_desiredAngle - m_currentAngle),
                                       Math.cos(m_desiredAngle - m_currentAngle));
        // Choose threshold depending on alliance (30° normally, 145° when on Red)
        // In these cases we want to aim a little closer. 
        double thresholdDeg = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) ? 140.0 : 30.0;
        // if the absolute angle to target is greater than the threshold, apply a 7-degree bias
        if (Math.abs(m_angleError) > Math.toRadians(thresholdDeg)) {
            double bias = Math.toRadians(7.75);//15.5);//7.0);
            if (m_angleError < 0) {
                m_desiredAngle -= bias; // compensate more negative
            } else {
                m_desiredAngle += bias; // compensate more positive
            }
            // recompute angleError after bias (optional)
            m_angleError = Math.atan2(Math.sin(m_desiredAngle - m_currentAngle), Math.cos(m_desiredAngle - m_currentAngle));
        }

    m_matchPercent = (1.0 - Math.min(1.0, Math.abs(m_angleError) / Math.PI)) * 100.0;

        boolean withinAimTolerance = fIsAutoAiming && lastTarget != null
                && IsRobotAlignedToTarget();
        SmartDashboard.putBoolean("AngleWithinTolerance", withinAimTolerance);
        SmartDashboard.putNumber("AngleError (deg)", Math.toDegrees(m_angleError));
        SmartDashboard.putNumber("AngleMatchPercent", m_matchPercent);
        // Publish normalized angles in 0-360 degrees for readability
        SmartDashboard.putNumber("Current Angle (deg)", radiansToDegrees360(m_currentAngle));
        SmartDashboard.putNumber("Desired Angle (deg)", radiansToDegrees360(m_desiredAngle));
        
    }

    /**
     * Return true when the robot is facing the given target within the provided
     * tolerance (radians).
     * false if target is null or the error is outside the tolerance.
     */
    private boolean IsRobotAlignedToTarget() {
       if (!fIsAutoAiming || lastTarget == null) 
            return false;
        
        // Return true when match is 96% or better
        return m_matchPercent >= 96.0;
       
    }

    public void ActivateAutoAim(Translation2d target)
    {
        lastTarget = target;
        fIsAutoAiming = true;
    }

    private double CalculateAnglePIDToTarget(Translation2d target) {
        if (!fIsAutoAiming || lastTarget == null) {
            return Double.NaN;
        }
        double rot = aimPID.calculate(m_currentAngle, m_desiredAngle);
        rot = MathUtil.clamp(rot, -MaxAngularRate, MaxAngularRate);

        return rot;
    }

    public void HitBrakes() {
        drivetrain.applyRequest(() -> brake);
    }

    private void TerminateAimCommand() {

        if (autoAimCommand != null && autoAimCommand.isScheduled()) {
            autoAimCommand.cancel();
            autoAimCommand = null;
            lastTarget = null;
        }
    }

    public void startTeleopTimer() {
        getLEDSystem().startCountdown(10);
        teleopTimer.stop();
        teleopTimer.reset();
        teleopTimer.start();
    }

    public void stopTeleopTimer() {
        teleopTimer.stop();
        teleopTimer.reset();
    }

    private void onTransitionChange(int eventNumber, int timeSeconds) {
        ledSystem.startCountdown(timeSeconds); // Start a 15-second countdown on the LEDs

        System.out.println("Teleop event " + eventNumber + " fired at " + timeSeconds + " seconds");
    }

    double speed = 1.0;

    private void configureBindings() {

        // compute rotational rate via PID toward lastTarget and use that rotation.
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> {
                    double vx = -driver.getLeftY() * MaxSpeed * driveScaler;
                    double vy = -driver.getLeftX() * MaxSpeed * driveScaler;

                    if (fIsAutoAiming && lastTarget != null && !driverOverride()) {
                        CalculateTargetAngles(); // Update the target angles and error for telemetry
                        double rot = CalculateAnglePIDToTarget(lastTarget);

                        return drive.withVelocityX(vx).withVelocityY(vy).withRotationalRate(rot);
                    } else {
                        return drive.withVelocityX(vx).withVelocityY(vy)
                                .withRotationalRate(-driver.getRightX() * MaxAngularRate);
                    }
                }));

        // Original drive code
        /*
         * drivetrain.setDefaultCommand(
         * // Drivetrain will execute this command periodically
         * drivetrain.applyRequest(() -> {
         * // translational drive from left stick (unchanged)
         * double vx = -driver.getLeftY() * MaxSpeed * driveScaler;
         * double vy = -driver.getLeftX() * MaxSpeed * driveScaler;
         * 
         * // default manual rotational control from right stick
         * double rotationalRate = -driver.getRightX() * MaxAngularRate;
         * return drive.withVelocityX(vx)
         * .withVelocityY(vy)
         * .withRotationalRate(rotationalRate);
         * }));
         */
        
         // If driver takes manual rotational control (right stick > 0.5) disable auto-aim.
        new Trigger(() -> Math.abs(driver.getRightX()) > 0.5)
            .onTrue(new InstantCommand(() -> fIsAutoAiming = false));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        new Trigger(() -> Math.abs(driver.getLeftY()) < deadzone
                && Math.abs(driver.getLeftX()) < deadzone
                && Math.abs(driver.getRightX()) < deadzone
                && IsRobotAlignedToTarget()  )
                .whileTrue(drivetrain.applyRequest(() -> brake));

        driver.b().whileTrue(drivetrain
                .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

        // Reset the field-centric heading on left bumper press.
        driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // emergency servo reset
        // driver.povUp().onTrue(new InstantCommand(() -> m_Shooter.SetHood(15)));
        driver.povDown().onTrue(new InstantCommand(() -> m_Shooter.SetHood(0)));

        // Firing logic
        driver.rightTrigger().whileTrue(new InstantCommand(() -> StartFiringSequence())
                //.alongWith(ExecuteAimCommand(m_Shooter.GetAllianceHub())));
                .alongWith(new InstantCommand(() -> ActivateAutoAim(m_Shooter.GetAllianceHub()))));
        driver.rightTrigger().onFalse(new InstantCommand(() -> CeaseFire()));

        new Trigger(() -> ReadyToFire())
                .onTrue(new InstantCommand(() -> {
                    // Measure the spin-up time from when WarmupShooter() was started
                    // (StartFiringSequence resets and starts m_warmupTimer) until ReadyToFire()
                    double spinup = m_warmupTimer.get();
                    SmartDashboard.putNumber("Shooter/SpinupTime", spinup);
                    System.out.println("Shooter spinup time: " + spinup + " seconds");
                    OpenFire();
                })); // activate the indexer once the shooter is warmed up and ready to fire.

        // intake logic
        driver.x().onTrue(new InstantCommand(() -> toggleIntake()));
        driver.a().whileTrue((new InstantCommand(() -> m_Intake.startIntake())));
        driver.a().onFalse((new InstantCommand(() -> m_Intake.stopIntake())));

        // Aiming logic
        driver.y().onTrue(new InstantCommand(() -> ActivateAutoAim(m_Shooter.GetAllianceHub())));

        // Phase transitions
        // Todo: implement warm up sequence on shooter based on phase
        // Fire once at 10 seconds
        new Trigger(() -> teleopTimer.hasElapsed(10.0))
                .onTrue(Commands.runOnce(() -> onTransitionChange(1, 30)));

        // Fire once at 35 seconds
        new Trigger(() -> teleopTimer.hasElapsed(35.0))
                .onTrue(Commands.runOnce(() -> onTransitionChange(2, 30)));
        // Fire once at 60 seconds
        new Trigger(() -> teleopTimer.hasElapsed(60.0))
                .onTrue(Commands.runOnce(() -> onTransitionChange(3, 30)));

        // Fire once at 85 seconds
        new Trigger(() -> teleopTimer.hasElapsed(85.0))
                .onTrue(Commands.runOnce(() -> onTransitionChange(4, 30)));

        // Fire once at 130 seconds
        // new Trigger(() -> teleopTimer.hasElapsed(130.0))
        // .onTrue(Commands.runOnce(() -> onTransitionChange(5, 130)));

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
                 .alongWith(new InstantCommand(() -> ActivateAutoAim(m_Shooter.GetAllianceHub()))));
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
                .alongWith(new InstantCommand(() -> m_Shooter.DeactivatePreShot()))
                .alongWith(new InstantCommand(() -> {
                    m_intakeDeployTimer.stop();
                    m_intakeCycleTimer.stop();
                    m_intakeCycleRunning = false;
                }))
                .alongWith(new InstantCommand(() -> m_Intake.deployIntake()))
                .alongWith(new InstantCommand(() -> m_Intake.stopIntake())));

        JoystickButton intakeReverseButton = new JoystickButton(operatorEmergency,
                Constants.EmergencyOperatorControls.ReverseIntake);
        intakeReverseButton.onTrue(new InstantCommand(() -> m_Intake.reverseIntake())
                .alongWith(new InstantCommand(() -> m_Indexer.reverseIndexer())));
        intakeReverseButton.onFalse(new InstantCommand(() -> m_Intake.stopIntake())
                .alongWith(new InstantCommand(() -> m_Indexer.stopIndexer())));

        // Passing Controls
    JoystickButton passZone4 = new JoystickButton(operatorStandard, Constants.StandardOperatorControls.Left);
    passZone4.whileTrue(new InstantCommand(() -> StartPassingSequence(PassingZone.Zone_Left))
        .alongWith(new InstantCommand(() -> ActivateAutoAim(m_Shooter.getPassingPoseLeftButton()))));
     passZone4.onFalse(new InstantCommand(() -> CeasePassing(PassingZone.Zone_Left)));

    JoystickButton passZone5 = new JoystickButton(operatorStandard, Constants.StandardOperatorControls.Right);
    passZone5.whileTrue(new InstantCommand(() -> StartPassingSequence(PassingZone.Zone_Right))
        .alongWith(new InstantCommand(() -> ActivateAutoAim(m_Shooter.getPassingPoseRightButton()))));
     passZone5.onFalse(new InstantCommand(() -> CeasePassing(PassingZone.Zone_Right)));

        JoystickButton warmUpButton = new JoystickButton(operatorEmergency,
                Constants.EmergencyOperatorControls.WarmupShooter);
        warmUpButton.onTrue(new InstantCommand(() -> m_Shooter.ActivatePreShot()));

        // warmUpButton.onFalse(new InstantCommand(() ->
        // m_Shooter.DeactivatePreShot()));
        // Sys ID code
        
          driver.povUp().whileTrue(drivetrain.applyRequest(() ->
          forwardStraight.withVelocityX(0.5).withVelocityY(0))
          );
         driver.povDown().whileTrue(drivetrain.applyRequest(() ->
          forwardStraight.withVelocityX(-0.5).withVelocityY(0))
          );
          
          // Run SysId routines when holding back/start and X/Y.
          // Note that each routine should be run exactly once in a single log.
          driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
          driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
          driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
          driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
         
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void showTeamColors() {
        ledSystem.showTeamColors();
    }
    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

}
