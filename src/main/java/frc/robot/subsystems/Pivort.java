
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.lang.Math;
import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

// import frc.robot.Constants.armConstants;
import frc.robot.Constants.pivotConstants;

public class Pivort extends SubsystemBase {
  /** Creates a new Pivort. */
  public Pivort() {}
  //3 variables to control the movement of the spinner
  double targetAngle = 0;  // Should be overwritten by manual mode on startup
  public double rotateSpeed;
  boolean autoTarget = false;
  public double difference;

   //Initiallizing the PIDs
  PIDController rotatePID = new PIDController(pivotConstants.PivotPIDkp, pivotConstants.PivotPIDki, pivotConstants.PivotPIDkd);
  SlewRateLimiter rotateLimiter = new SlewRateLimiter(16); //Slew rate limiters act as limits to acceleration
  SlewRateLimiter continueRotateLimiter = new SlewRateLimiter(0.5);

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Auto target", autoTarget);
    this.difference = rotateToPosition(RobotContainer.isRedAlliance ? Constants.fieldConstants.RedFieldHub : Constants.fieldConstants.BlueFieldHub);
    rotateSpeed = rotate(difference);
  }

  //Takes in the controller input and decides whether to just return that or whether to return the pid values to point it towards hub
  public double findRotateSpeed(double manualSpeed){
    if (autoTarget) {
      RobotContainer.xbox1.setRumble(RumbleType.kBothRumble, 1);
      return RobotContainer.Deadzone(rotateSpeed, .1);
    }
    else {
      RobotContainer.xbox1.setRumble(RumbleType.kBothRumble, 0);
      return RobotContainer.Deadzone(manualSpeed);
    }
  }

  //Takes in the difference in angle from the robot to the center of the hub and returns the speed
  public double rotate(double difference) {
      if (difference > 180) {
        difference -= 360;
      }
      
      if (difference < -180) {
        difference += 360;
      }
    
    SmartDashboard.putNumber("Difference", difference);
    if (Double.isNaN(difference)) {
      return 0;
    }
      return rotateLimiter.calculate(rotatePID.calculate(difference)); 
  }


  public double rotateToPosition(Pose2d targetPose) {
    double x = targetPose.getX();
    double y = targetPose.getY(); 
    //x,y,z is target position; z is vertical(depending on coord system it might be different)
    Pose2d pose = RobotContainer.vision.getFieldPose();
    
    return(pose.getRotation().getDegrees() - Math.toDegrees(Math.atan2(x - pose.getX(),y - pose.getY())) + 3);
  }

  public void setAutoTarget(boolean autoTarget) {
    this.autoTarget = autoTarget;
  }

  public double getDifferenceToTarget() {
    return this.difference;
  }
}