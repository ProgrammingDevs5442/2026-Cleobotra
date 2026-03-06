
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
  boolean manualRotateMode = false;
  double targetAngle = 0;  // Should be overwritten by manual mode on startup
  double rotateSpeed;
  double targetAutoRotate = 0;
  double robotRotation = 0;
  double manualDifference;
  double trackedDifference;
  boolean continuing = false;
  boolean autoTarget = false;
  double continueAngle = 0;
  ArrayList<String> output = new ArrayList<>();

   //Initiallizing the PIDs
  PIDController rotatePID = new PIDController(pivotConstants.PivotPIDkp, pivotConstants.PivotPIDki, pivotConstants.PivotPIDkd);
  SlewRateLimiter rotateLimiter = new SlewRateLimiter(16);
  SlewRateLimiter continueRotateLimiter = new SlewRateLimiter(0.5);

  @Override
  public void periodic() {

    // SmartDashboard.putNumber("Pivot Raw Encoder", RobotContainer.rotateMotor.getPosition().getValueAsDouble());
    // SmartDashboard.putNumber("Pivot Degrees", Math.toDegrees(getAngle()));
    // SmartDashboard.putNumber("Pivot Target Angle", targetAngle);
    // SmartDashboard.putBoolean("Continuing", continuing);
    // SmartDashboard.putStringArray("output", output.toArray(new String[0]));
    // SendableRegistry.setName(rotatePID, "Pivot", "PivotPID");
    // SmartDashboard.putBoolean("Auto target", autoTarget);
    
    
    // SendableRegistry.setName(RobotContainer.rotateMotor, "Rotate speed");

    //double difference =  trackedDifference;
    double difference = rotateToPosition(RobotContainer.isRedAlliance ? Constants.fieldConstants.RedFieldHub : Constants.fieldConstants.BlueFieldHub);//TODO make flip with sides
   
    rotateSpeed = rotate(difference);
  }

  public double getDifference() {
    return this.trackedDifference;
  }

  public double findRotateSpeed(double manualSpeed){
    // SmartDashboard.putNumber("Pivot Speed", rotateSpeed);
    // SmartDashboard.putNumber("manualRotateSpeed", manualSpeed);
    if (autoTarget) {
      return RobotContainer.Deadzone(rotateSpeed, .1);
    }
    else {
      return RobotContainer.Deadzone(manualSpeed);
    }
  }

  public void calculateRotateDifference(double targetAngle){
      manualDifference = Math.toDegrees(getAngle()) - targetAngle;
  }

  public double rotate(double difference) {
    // Positive difference means a more negative pivot angle
    // if (difference == 0) return 0;
    if (!continuing) {
      if (difference > 180) {
        difference -= 360;
      }
      
      if (difference < -180) {
        difference += 360;
      }
    }
    
    SmartDashboard.putNumber("Difference", difference);
    if (Double.isNaN(difference)) {
      return 0;
    }
      return rotateLimiter.calculate(rotatePID.calculate(difference)); 
  }

  public void setAutoRotate(double trackedDifference) {
    this.trackedDifference = trackedDifference;
  }

  public void setTargetAngle(double targetAngle) {
    if (Double.isNaN(targetAngle)) {
      targetAngle=0;
    }
    this.targetAngle = targetAngle;
    // SmartDashboard.putNumber("Pivot Target Angle", targetAngle);
    // SmartDashboard.putNumber("Pivot Speed", rotateSpeed);
   
  }

  public double getAngle(){
    //Takes in value from encoder in rotations and returns the value in Degrees
    return 0;//(RobotContainer.rotateMotor.getPosition().getValueAsDouble() * pivotConstants.PivotTableRatio * pivotConstants.PivotMotorRatio * 2 * Math.PI);
  }

  public double rotateToPosition(Pose2d targetPose) {
    double x = targetPose.getX();
    double z = targetPose.getY(); 
    //x,y,z is target position; y is vertical
    //speed is a constant factor, 1.15 (might want to change)
    Pose2d pose = RobotContainer.vision.getFieldPose();
    
    // SmartDashboard.putNumber("Field Angle to hub", Math.toDegrees(Math.atan2(x - pose.getX(),z - pose.getY())));
    // SmartDashboard.putNumber("Relative Angle to hub", pose.getRotation().getDegrees() - Math.toDegrees(Math.atan2(x - pose.getX(),z - pose.getY())));
    
    return(pose.getRotation().getDegrees() - Math.toDegrees(Math.atan2(x - pose.getX(),z - pose.getY())));
  }

  public void TagTracking(double TagID) {
    

  }

  public void setAutoTarget(boolean autoTarget) {
    this.autoTarget = autoTarget;
  }

  public void manualMode(boolean manualRotateMode) {
    this.manualRotateMode = manualRotateMode;
  }
}