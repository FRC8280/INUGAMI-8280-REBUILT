// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
//import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

//import dev.doglog.DogLog;
//import dev.doglog.DogLogOptions;

public class Robot extends TimedRobot {

    private NetworkTableEntry seedGyroButton;
    // Track last state for edge detection
    private boolean lastSeedState = false;

    private boolean opModeStarted = false;
    private boolean secondLimeLight = true;
    private double lastLoopTime = Timer.getFPGATimestamp();
    private final Field2d m_field = new Field2d();

    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();

    private final boolean kUseLimelight = true;
    private boolean gyroSeeded = false;

    public Robot() {
        m_robotContainer = new RobotContainer();
        // DogLog.setOptions(new DogLogOptions().withCaptureDs(true));
        SmartDashboard.putData("Field", m_field);
    }

    @Override
    public void robotInit() {

        // This publishes the button key so dashboards can show it
        SmartDashboard.putBoolean("SeedGyro", false);

    }

    private boolean shouldAcceptMT2(LimelightHelpers.PoseEstimate estimate, double omegaRps) {
        if (estimate == null) {
            return false;
        }

        // Must see at least one tag
        if (estimate.tagCount <= 0) {
            return false;
        }

        // Reject while spinning fast
        if (Math.abs(omegaRps) >= 2.0) {
            return false;
        }

        // Reject very far measurements
        if (estimate.avgTagDist > 7.0) {
            return false;
        }

        // Far single-tag MT2 is usually sketchy
        if (estimate.avgTagDist > 4.0 && estimate.tagCount < 2) {
            return false;
        }

        return true;
    }

    private boolean addFilteredMT2Vision(LimelightHelpers.PoseEstimate estimate, double omegaRps) {
        if (!shouldAcceptMT2(estimate, omegaRps)) {
            return false;
        }

        double dist = estimate.avgTagDist;

        // Trust close tags more, far tags less
        double xyStdDev = 0.35 + (dist * 0.25);

        // If only one tag, trust it less
        if (estimate.tagCount == 1) {
            xyStdDev += 0.75;
        }

        m_robotContainer.drivetrain.setVisionMeasurementStdDevs(
                VecBuilder.fill(xyStdDev, xyStdDev, 9999999));

        m_robotContainer.drivetrain.addVisionMeasurement(
                estimate.pose,
                estimate.timestampSeconds);
        return true;
    }

    @Override
    public void robotPeriodic() {

        double now = Timer.getFPGATimestamp();
        double loopTime = now - lastLoopTime;
        lastLoopTime = now;
        SmartDashboard.putNumber("Robot/LoopTime", loopTime);

        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();

        /*
         * if (m_robotContainer != null)
         * m_robotContainer.periodic();
         */ // dead code

        if (kUseLimelight && !gyroSeeded) {
            seedGyro(false);
        }

        if (kUseLimelight && gyroSeeded) {

            var driveState = m_robotContainer.drivetrain.getState();
            double headingDeg = driveState.Pose.getRotation().getDegrees();
            double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

            LimelightHelpers.SetIMUAssistAlpha("limelight", 0.001);
            LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
            LimelightHelpers.SetIMUAssistAlpha("limelight-rear", 0.001);
            LimelightHelpers.SetRobotOrientation("limelight-rear", headingDeg, 0, 0, 0, 0, 0);

            var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            var llRearMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-rear");

            // Additional filtering
            /*
             * addFilteredMT2Vision(llMeasurement, omegaRps);
             * if (Constants.kVerboseDashboard) {
             * SmartDashboard.putNumber("Front LL-X", llMeasurement.pose.getX());
             * SmartDashboard.putNumber("Front LL-Y", llMeasurement.pose.getY());
             * SmartDashboard.putNumber("Front LL-TagCount", llMeasurement.tagCount);
             * }
             * 
             * if (secondLimeLight && !opModeStarted) {
             * addFilteredMT2Vision(llRearMeasurement, omegaRps);
             * SmartDashboard.putNumber("Rear LL-X", llMeasurement.pose.getX());
             * SmartDashboard.putNumber("Rear LL-Y", llMeasurement.pose.getY());
             * SmartDashboard.putNumber("Rear LL-TagCount", llMeasurement.tagCount);
             * }
             */

            if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {

                m_robotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,
                        .5, 9999999));
                m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose,
                        llMeasurement.timestampSeconds);

                if (Constants.kVerboseDashboard) {
                    SmartDashboard.putNumber("Front LL-X", llMeasurement.pose.getX());
                    SmartDashboard.putNumber("Front LL-Y", llMeasurement.pose.getY());
                    SmartDashboard.putNumber("Front LL-TagCount", llMeasurement.tagCount);

                }
            } 
            /*
            else if (secondLimeLight && !opModeStarted) { // leon turn this off ifissues
                LimelightHelpers.SetIMUAssistAlpha("limelight-rear", 0.001);
                LimelightHelpers.SetRobotOrientation("limelight-rear", headingDeg, 0, 0, 0,
                        0, 0);
                llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-rear");
                // addFilteredMT2Vision(llMeasurement, omegaRps);
                if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {

                    m_robotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.5,
                            .5, 9999999));
                    m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose,
                            llMeasurement.timestampSeconds);

                    if (Constants.kVerboseDashboard) {
                        SmartDashboard.putNumber("Rear LL-X", llMeasurement.pose.getX());
                        SmartDashboard.putNumber("Rear LL-Y", llMeasurement.pose.getY());
                        SmartDashboard.putNumber("Rear LL-TagCount", llMeasurement.tagCount);
                    }
                }
            }*/

        }

        m_field.setRobotPose(m_robotContainer.drivetrain.getState().Pose);

        if (Constants.kVerboseDashboard) {
            SmartDashboard.putNumber("Current Drive X",
                    m_robotContainer.drivetrain.getState().Pose.getX());
            SmartDashboard.putNumber("Current Drive Y",
                    m_robotContainer.drivetrain.getState().Pose.getY());
            SmartDashboard.putNumber("Current Yaw",
                    m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees());
        }

        boolean currentState = SmartDashboard.getBoolean("SeedGyro", false);

        // Detect rising edge (button just pressed)
        if (currentState && !lastSeedState) {
            seedGyroButton();
            SmartDashboard.putBoolean("SeedGyro", false);
        }

        lastSeedState = currentState;

    }

    // boolean hack = false;
    public void seedGyroButton() {

        boolean hack = false;
        if (gyroSeeded)
            hack = true;
        gyroSeeded = false;
        seedGyro(false);
    }

    public void seedGyro(boolean onlyFront) {
        // Publish telemetry when seeding is attempted
        SmartDashboard.putString("SeedGyro/LastAction", "seedGyro called");
        SmartDashboard.putNumber("SeedGyro/LastAttemptTime", Timer.getFPGATimestamp());

        if (gyroSeeded)
            return;

        LimelightHelpers.setLimelightNTDouble("limelight", "throttle_set", 0);
        LimelightHelpers.SetIMUMode("limelight", 4);

        LimelightHelpers.setLimelightNTDouble("limelight-rear", "throttle_set", 0);
        LimelightHelpers.SetIMUMode("limelight-rear", 4);

        /*
         * try {
         * Thread.sleep(500);
         * } catch (InterruptedException e) {
         * Thread.currentThread().interrupt();
         * }
         */

        var mt1rear = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-rear");
        var mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

        if (mt1rear != null && mt1rear.tagCount >= 1) {
            SmartDashboard.putString("SeedGyro/LastAction", "MT1 rear multi-tag pose");
            SmartDashboard.putNumber("SeedGyro/LastTagCount", mt1rear.tagCount);
            seedFromPose(mt1rear.pose);
        } else if (mt1 != null && mt1.tagCount >= 1) {
            SmartDashboard.putString("SeedGyro/LastAction", "MT1 front multi-tag pose");
            SmartDashboard.putNumber("SeedGyro/LastTagCount", mt1.tagCount);
            seedFromPose(mt1.pose);
        } else {
            SmartDashboard.putString("SeedGyro/LastAction", "no valid multi-tag MT1 pose available");
            SmartDashboard.putNumber("SeedGyro/LastTagCount",
                    (mt1rear != null ? mt1rear.tagCount : 0) + (mt1 != null ? mt1.tagCount : 0));
        }

        /*
         * else {
         * // Todo test this code.
         * System.out.println("seedGyro: insufficient tags (" + mt1.tagCount +
         * "), not seeding.");
         * if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
         * m_robotContainer.drivetrain.seedFieldCentric(
         * new Rotation2d(Math.toRadians(mt1.pose.getRotation().getDegrees() - 180)));
         * 
         * }
         */
    }

    private void seedFromPose(Pose2d pose) {
        gyroSeeded = true;
        SmartDashboard.putString("SeedGyro/LastAction", "seedFromPose");
        SmartDashboard.putNumber("SeedGyro/LastSeedYawDeg", pose.getRotation().getDegrees());
        double yaw;
        yaw = pose.getRotation().getDegrees();
        // if(hack)
        // yaw+=180;

        m_robotContainer.drivetrain.getPigeon2().setYaw(yaw);
        m_robotContainer.drivetrain.resetPose(pose);
        m_robotContainer.drivetrain.seedFieldCentric(pose.getRotation());
    }

    @Override
    public void disabledInit() {
        opModeStarted = false;
        // DogLog.log("RoboRIO ID", RobotController.getSerialNumber());
        seedGyro(false);
        LimelightHelpers.SetIMUMode("limelight", 1); // Seed internal IMU
        LimelightHelpers.setLimelightNTDouble("limelight", "throttle_set", 200);

        LimelightHelpers.SetIMUMode("limelight-rear", 1); // Seed internal IMU
        LimelightHelpers.setLimelightNTDouble("limelight-rear", "throttle_set", 0);

        m_robotContainer.showTeamColors();

    }

    private double lastSeedAttempt = 0;

    @Override
    public void disabledPeriodic() {
        /*
         * double now = Timer.getFPGATimestamp();
         * 
         * if (!gyroSeeded && now - lastSeedAttempt > 1.0) {
         * seedGyro();
         * lastSeedAttempt = now;
         * }
         */
    }

    @Override
    public void disabledExit() {
        m_robotContainer.stopTeleopTimer();
    }

    @Override
    public void autonomousInit() {

        opModeStarted = true;
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }

        LimelightHelpers.setLimelightNTDouble("limelight", "throttle_set", 0);
        LimelightHelpers.SetIMUMode("limelight", 4);

        LimelightHelpers.setLimelightNTDouble("limelight-rear", "throttle_set", 0);
        LimelightHelpers.SetIMUMode("limelight-rear", 4);

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
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }

        opModeStarted = true;
        // m_robotContainer.getLEDSystem().startCountdown(30);
        m_robotContainer.startTeleopTimer();

        LimelightHelpers.setLimelightNTDouble("limelight", "throttle_set", 0);
        LimelightHelpers.SetIMUMode("limelight", 4);

        LimelightHelpers.setLimelightNTDouble("limelight-rear", "throttle_set", 0);
        LimelightHelpers.SetIMUMode("limelight-rear", 4);

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