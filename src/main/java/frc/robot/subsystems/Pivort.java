
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

    SmartDashboard.putNumber("Pivot Raw Encoder", RobotContainer.rotateMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Pivot Degrees", Math.toDegrees(getAngle()));
    SmartDashboard.putNumber("Pivot Target Angle", targetAngle);
    SmartDashboard.putNumber("Pivot Speed", rotateSpeed);
    SmartDashboard.putBoolean("Continuing", continuing);
    SmartDashboard.putStringArray("output", output.toArray(new String[0]));
    SendableRegistry.setName(rotatePID, "Pivot", "PivotPID");
    
    SendableRegistry.setName(RobotContainer.rotateMotor, "Rotate speed");

    // double difference =  trackedDifference;

    // if (difference == 0 && continuing) {
    //   difference = Math.toDegrees(getAngle()) - continueAngle;
    //   continuing = (Math.abs(difference) > 1) && (Math.abs(difference) < 400);
    //   if (!continuing) {
    //     difference = 0;
    //   }
    // } else {
    //   continuing = false;
    // }
    // rotateSpeed = rotate(difference);
    // // if (manualRotateMode) {
    // //   rotate(manualDifference);
    // // } else {
    // //   rotate(trackedDifference);
    // // }

    // robotRotation = RobotContainer.joystick.getRightX() * (Constants.driveConstants.MaxAngularRate);
    // if (Math.abs(robotRotation) < Constants.driveConstants.RotationalDeadband){
    //   robotRotation = 0;
    // } else {
    //   robotRotation /= (20.247 * Math.PI);
    // }

    //At all times, set the motor to the speed given
    // RobotContainer.rotateMotor.set(rotateSpeed);//-  4 * robotRotation);
    
  }

  public double findRotateSpeed(double manualSpeed){
    if (autoTarget) {
      return rotateSpeed;
    }
    else {
      return manualSpeed;
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
    
    // double angle = Math.toDegrees(getAngle()) - difference;
    // if (continuing) System.out.println("2Angle: " + angle + "    Difference: " + difference);
    // if (angle > Constants.pivotConstants.PivotHighLimit) {
    //   System.out.println("[UPPER LIMIT] Angle: " + angle + "    Difference: " + difference);
    //   difference = 0;
    //   // difference += 360;
    //   // continuing = true;
    //   // continueAngle = Math.toDegrees(getAngle()) - difference;
    // } else if (angle < Constants.pivotConstants.PivotLowLimit) {
    //   System.out.println("[LOWER LIMIT] Angle: " + angle + "    Difference: " + difference);
    //   difference = 0;
    //   // difference -= 360;
    //   // continuing = true;
    //   // continueAngle = Math.toDegrees(getAngle()) - difference;
    // }
    
    SmartDashboard.putNumber("Difference", difference);
    // if (continuing) System.out.println("Continuing: " + continuing + "   Difference: " + difference + "   ContinueAngle: " + continueAngle);
    // output.add("Difference: " + difference + " | Continuing: " + continuing + " | ContinueAngle: " + continueAngle);
    if (Double.isNaN(rotateSpeed)) {
      return 0;
    }
    // rotateSpeed = rotateLimiter.calculate(rotatePID.calculate(difference));
    // if (continuing) {
    //   return continueRotateLimiter.calculate(rotatePID.calculate(difference)); //TODO double check the limiter
    // } else {
      return rotateLimiter.calculate(rotatePID.calculate(difference)); 
    // }
  }

  public void setAutoRotate(double trackedDifference) {
    this.trackedDifference = trackedDifference;
  }

  public void setTargetAngle(double targetAngle) {
    if (Double.isNaN(targetAngle)) {
      targetAngle=0;
    }
    this.targetAngle = targetAngle;
    SmartDashboard.putNumber("Pivot Target Angle", targetAngle);
    SmartDashboard.putNumber("Pivot Speed", rotateSpeed);
   
  }

  public double getAngle(){
    //Takes in value from encoder in rotations and returns the value in Degrees
    return (RobotContainer.rotateMotor.getPosition().getValueAsDouble() * pivotConstants.PivotTableRatio * pivotConstants.PivotMotorRatio * 2 * Math.PI);
  }

  public void shootAtPosition(double x, double y, double z, double speed) {
    //x,y,z is target position; y is vertical
    //speed is a constant factor, 1.15 (might want to change)
    Pose2d pose = RobotContainer.vision.getFieldPose();

    setAutoRotate((180 - Math.atan2(z - pose.getY(), x - pose.getX())) - (pose.getRotation().getDegrees() + getAngle()));

    // double dist = Math.sqrt(Math.pow(x - pose.getX(),2) + Math.pow(z - pose.getY(),2));
    // double ys = Constants.shooterConstants.HeightOfShooter;
    // double theta = Math.toRadians(Constants.shooterConstants.AngleOfShooter);
    // calculatedShootVelocity = speed * ((4*dist))/(Math.sqrt(-(Math.cos(theta)*((y-ys)*Math.cos(theta)-Math.sin(theta)*dist))));
    
    // calcMotorAngVelo = calculatedShootVelocity/(Constants.shooterConstants.DiameterOfWheel/2);
    // shootSpeed = calcMotorAngVelo/(Constants.pivotConstants.MaxRPMPivot * Constants.pivotConstants.RPMToRadPS * Constants.pivotConstants.MotorTransferEfficency);
    
  }

  // public void shootSpeed(double speed){
  //   // shootSpeed = speed * Constants.pivotConstants.DistanceToShootSpeedMultiplier;
  //   double xs = RobotContainer.turretVision.getDistanceToTag() * Constants.shooterConstants.MetersToFeet;
  //   double ys = Constants.shooterConstants.HeightOfShooter;
  //   double theta = Math.toRadians(Constants.shooterConstants.AngleOfShooter);
  //   calculatedShootVelocity = speed * ((4*xs))/(Math.sqrt(-(Math.cos(theta)*((Constants.fieldConstants.HeightOfHub-ys)*Math.cos(theta)-Math.sin(theta)*xs))));
    
  //   calcMotorAngVelo = calculatedShootVelocity/(Constants.shooterConstants.DiameterOfWheel/2);
  //   shootSpeed = calcMotorAngVelo/(Constants.pivotConstants.MaxRPMPivot * Constants.pivotConstants.RPMToRadPS * Constants.pivotConstants.MotorTransferEfficency);
  //   SmartDashboard.putNumber("shootSpeed", shootSpeed);
  //   SmartDashboard.putNumber("Calculated Shoot Speed", calculatedShootVelocity);
  // }
  public void TagTracking(double TagID) {
    

  }

  public void setAutoTarget(boolean autoTarget) {
    this.autoTarget = autoTarget;
  }

  public void manualMode(boolean manualRotateMode) {
    this.manualRotateMode = manualRotateMode;
  }
}