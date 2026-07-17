// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; You can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private static final boolean USE_LIMELIGHT = true;

    private final RobotContainer m_robotContainer;
    private final VisionManager m_visionManager;
    private final Field2d m_field = new Field2d();

    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();

    private Command m_autonomousCommand;
    private double lastLoopTime = Timer.getFPGATimestamp();
    private boolean lastSeedButtonState = false;

    public Robot() {
        m_robotContainer = new RobotContainer();
        m_visionManager =
                new VisionManager(
                        m_robotContainer.drivetrain,
                        USE_LIMELIGHT);
        SmartDashboard.putData("Field", m_field);
    }

    @Override
    public void robotInit() {
        SmartDashboard.putBoolean("SeedGyro", false);
        m_visionManager.initialize();
    }

    @Override
    public void robotPeriodic() {
        double now = Timer.getFPGATimestamp();
        SmartDashboard.putNumber(
                "Robot/LoopTime",
                now - lastLoopTime);
        lastLoopTime = now;

        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();

        m_visionManager.periodic();
        handleSeedButton();

        var pose = m_robotContainer.drivetrain.getState().Pose;
        m_field.setRobotPose(pose);

        if (Constants.kVerboseDashboard) {
            SmartDashboard.putNumber(
                    "Current Drive X",
                    pose.getX());
            SmartDashboard.putNumber(
                    "Current Drive Y",
                    pose.getY());
            SmartDashboard.putNumber(
                    "Current Yaw",
                    pose.getRotation().getDegrees());
            SmartDashboard.putBoolean(
                    "Vision/GyroSeeded",
                    m_visionManager.isGyroSeeded());
        }
    }

    private void handleSeedButton() {
        boolean currentState =
                SmartDashboard.getBoolean("SeedGyro", false);

        if (currentState && !lastSeedButtonState) {
            m_visionManager.requestManualReseed();
            SmartDashboard.putBoolean("SeedGyro", false);
        }

        lastSeedButtonState = currentState;
    }

    @Override
    public void disabledInit() {
        m_visionManager.onDisabledInit();
        //m_robotContainer.showTeamColors();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
        m_robotContainer.stopTeleopTimer();
    }

    @Override
    public void autonomousInit() {
        m_visionManager.onEnabledInit();

        m_autonomousCommand =
                m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance()
                    .schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance()
                    .cancel(m_autonomousCommand);
        }

        m_visionManager.onEnabledInit();
        m_robotContainer.startTeleopTimer();
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
        m_robotContainer.stopTeleopTimer();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
