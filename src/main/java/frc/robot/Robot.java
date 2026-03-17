// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
//test
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import com.ctre.phoenix6.hardware.Pigeon2;

public class Robot extends TimedRobot {
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

    public Robot() {
        m_robotContainer = new RobotContainer();
        DogLog.setOptions(new DogLogOptions().withCaptureDs(true));
        SmartDashboard.putData("Field", m_field);
    }

    @Override
    public void robotPeriodic() {

        double now = Timer.getFPGATimestamp();
        double loopTime = now - lastLoopTime;
        lastLoopTime = now;
        SmartDashboard.putNumber("Robot/LoopTime", loopTime);

        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();

        if(m_robotContainer != null) 
            m_robotContainer.periodic();
        /*
         * This example of adding Limelight is very simple and may not be sufficient for on-field use.
         * Users typically need to provide a standard deviation that scales with the distance to target
         * and changes with number of tags available.
         *
         * This example is sufficient to show that vision integration is possible, though exact implementation
         * of how to use vision should be tuned per-robot and to the team's specification.
         */


        if (kUseLimelight) {
            
            var driveState = m_robotContainer.drivetrain.getState();
            double headingDeg = driveState.Pose.getRotation().getDegrees();
            double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

            LimelightHelpers.SetIMUAssistAlpha("limelight", 0.001);
            LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
            var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
                m_robotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
                m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
            }

            if(secondLimeLight)
            {
                LimelightHelpers.SetIMUAssistAlpha("limelight-rear", 0.001);
                LimelightHelpers.SetRobotOrientation("limelight-rear", headingDeg, 0, 0, 0, 0, 0);
                llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-rear");
                if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
                    m_robotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
                    m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
            }
            }
        }

     m_field.setRobotPose(m_robotContainer.drivetrain.getState().Pose);

     if(Constants.kVerboseDashboard){
        SmartDashboard.putNumber("Current Drive X",
        m_robotContainer.drivetrain.getState().Pose.getX());
        SmartDashboard.putNumber("Current Drive Y",
        m_robotContainer.drivetrain.getState().Pose.getY());
        SmartDashboard.putNumber("Current Yaw",
        m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees());
     }
        
    }
    
    public void seedGyro()
    {
        //Todo: if this is null check the second camera
        var mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        if(mt1!=null){
            if(mt1.tagCount >=2){
                m_robotContainer.drivetrain.getPigeon2().setYaw(mt1.pose.getRotation().getDegrees());
                m_robotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0,0,Math.toRadians(0)));    
                m_robotContainer.drivetrain.addVisionMeasurement(mt1.pose, Utils.fpgaToCurrentTime(mt1.timestampSeconds));
               // if( DriverStation.getAllia nce().get() == DriverStation.Alliance.Red)
                //    m_robotContainer.drivetrain.seedFieldCentric(new Rotation2d(Math.toRadians(mt1.pose.getRotation().getDegrees()-180)));
              //  else
                    m_robotContainer.drivetrain.seedFieldCentric(mt1.pose.getRotation());
            }
        }
    }
    @Override
    public void disabledInit() {
        DogLog.log("RoboRIO ID", RobotController.getSerialNumber());
        //seedGyro();
        LimelightHelpers.SetIMUMode("limelight", 1); // Seed internal IMU
        LimelightHelpers.setLimelightNTDouble("limelight", "throttle_set", 200);

        if(secondLimeLight)
        {
            LimelightHelpers.SetIMUMode("limelight-rear", 1); // Seed internal IMU
            LimelightHelpers.setLimelightNTDouble("limelight-rear", "throttle_set", 200);
        }
    }

    @Override
    public void disabledPeriodic() {}
    

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {


        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }

        LimelightHelpers.setLimelightNTDouble("limelight", "throttle_set", 0);
        LimelightHelpers.SetIMUMode("limelight", 4); 

        if(secondLimeLight)
        {
            LimelightHelpers.setLimelightNTDouble("limelight-rear", "throttle_set", 0);
            LimelightHelpers.SetIMUMode("limelight-rear", 4);  
        }
    }

    @Override
    public void autonomousPeriodic() {}
    

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
        LimelightHelpers.setLimelightNTDouble("limelight", "throttle_set", 0);
        LimelightHelpers.SetIMUMode("limelight", 4); 

        if(secondLimeLight)
        {
            LimelightHelpers.setLimelightNTDouble("limelight-rear", "throttle_set", 0);
            LimelightHelpers.SetIMUMode("limelight-rear", 4); 
        }
    }

    @Override
    public void teleopPeriodic() {}


    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}

}