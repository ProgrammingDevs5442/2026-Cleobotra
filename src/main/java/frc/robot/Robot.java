// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.AudioConfigs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Vision.Vision;

import com.ctre.phoenix6.*;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final boolean kUseLimelight = true;
  public static boolean isAutonomous;

  public Robot() {
    enableLiveWindowInTest(true);
    m_robotContainer = new RobotContainer();
    SignalLogger.enableAutoLogging(false);
    SignalLogger.stop();
    // AudioConfigs.withAllowMusicDurDisable(true);
  }
  

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // if (!RobotContainer.hasFieldOriented && RobotContainer.vision.hasTarget()) {
    //   RobotContainer.drivetrain.resetRotation(RobotContainer.vision.getFieldPose().getRotation());
    //   RobotContainer.hasFieldOriented = true;
    // }

    /*
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */

//     public final AudioConfigs withAllowMusicDurDisable(boolean newAllowMusicDurDisable)
// 114    {
// 115        AllowMusicDurDisable = newAllowMusicDurDisable;
// 116        return this;
// 117    }

    if (kUseLimelight) {
      var driveState = m_robotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees();
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      LimelightHelpers.SetRobotOrientation("limelight-mason", headingDeg, 0, 0, 0, 0, 0);

      var llMeasurement = RobotContainer.vision.getFieldPose();
      var llTimeMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-mason");
      if (llMeasurement != null && RobotContainer.vision.hasTarget() && omegaRps < 2.0) {
        m_robotContainer.drivetrain.addVisionMeasurement(llTimeMeasurement.pose, Utils.fpgaToCurrentTime(llTimeMeasurement.timestampSeconds));
      }
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // isAutonomous();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    isAutonomous = true;
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    isAutonomous = false;
  }

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
