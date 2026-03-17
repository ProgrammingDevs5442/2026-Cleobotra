
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
  SlewRateLimiter rotateLimiter = new SlewRateLimiter(16);
  SlewRateLimiter continueRotateLimiter = new SlewRateLimiter(0.5);

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("Auto target", autoTarget);
    
    this.difference = rotateToPosition(RobotContainer.isRedAlliance ? Constants.fieldConstants.RedFieldHub : Constants.fieldConstants.BlueFieldHub);
   
    rotateSpeed = rotate(difference);
  }


  public double findRotateSpeed(double manualSpeed){
    if (autoTarget) {
      return RobotContainer.Deadzone(rotateSpeed, .1);
    }
    else {
      return RobotContainer.Deadzone(manualSpeed);
    }
  }


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
    //x,y,z is target position; y is vertical
    //speed is a constant factor, 1.15 (might want to change)
    Pose2d pose = RobotContainer.vision.getFieldPose();
    
    return(pose.getRotation().getDegrees() - Math.toDegrees(Math.atan2(x - pose.getX(),y - pose.getY())));
  }

  public void setAutoTarget(boolean autoTarget) {
    this.autoTarget = autoTarget;
  }

  public double getDifferenceToTarget() {
    return this.difference;
  }
}