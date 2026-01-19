
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.lang.Math;
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
  double calculatedShootVelocity = 0;
  double calcMotorAngVelo = 0;
  double shootSpeed;
  double targetAutoRotate = 0;
  double robotRotation = 0;

   //Initiallizing the PIDs
  PIDController rotatePID = new PIDController(pivotConstants.PivotPIDkp, pivotConstants.PivotPIDki, pivotConstants.PivotPIDkd);
  SlewRateLimiter rotateLimiter = new SlewRateLimiter(16);

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Pivot Raw Encoder", RobotContainer.rotateMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Pivot Degrees", Math.toDegrees(getAngle()));
    SmartDashboard.putNumber("Pivot Target Angle", targetAngle);
    SmartDashboard.putNumber("Pivot Speed", rotateSpeed);
    SmartDashboard.putNumber("dX", RobotContainer.turretVision.TagTracking());
    SmartDashboard.putNumber("dz", RobotContainer.turretVision.getDistanceToTag());


    SendableRegistry.setName(rotatePID, "Pivot", "PivotPID");
    
    SendableRegistry.setName(RobotContainer.rotateMotor, "Rotate speed");
    
    SendableRegistry.setName(RobotContainer.shootMotor, "Shoot speed");
    if (manualRotateMode) {
      rotate(targetAngle);
    }
    else {
      autoRotate();
    }

    robotRotation = RobotContainer.joystick.getRightX() * (Constants.driveConstants.MaxAngularRate);
    if (robotRotation < Constants.driveConstants.RotationalDeadband){
      robotRotation = 0;
    } else {
      robotRotation /= (20.247*Math.PI);
    }

    //At all times, set the motor to the speed given
    RobotContainer.rotateMotor.set(rotateSpeed - robotRotation);
    RobotContainer.shootMotor.set(shootSpeed); 
  }

  public void rotate(double targetAngle) {
    double difference = Math.toDegrees(getAngle()) - targetAngle;
    
    SmartDashboard.putNumber("Difference", difference);

    if (difference > 180) {
      difference -= 360;
    }

    if (difference < -180) {
      difference += 360;
    }

    if (Double.isNaN(rotateSpeed)) {
      rotateSpeed = 0;
    }
    rotateSpeed = rotateLimiter.calculate(rotatePID.calculate(difference));
  }

  public void autoRotate() {
    rotateSpeed = rotateLimiter.calculate(rotatePID.calculate(targetAutoRotate));
  }

  public void setAutoRotate(double trackedDifference) {
    targetAutoRotate = trackedDifference;
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
    Pose2d pose = RobotContainer.vision.getFieldPose();

    setAutoRotate((180 - Math.atan2(z - pose.getY(), x - pose.getX())) - (pose.getRotation().getDegrees() + getAngle()));

    double dist = Math.sqrt(Math.pow(x - pose.getX(),2) + Math.pow(z - pose.getY(),2));
    double ys = Constants.pivotConstants.HeightOfShooter;
    double theta = Math.toRadians(Constants.pivotConstants.AngleOfShooter);
    calculatedShootVelocity = speed * ((4*dist))/(Math.sqrt(-(Math.cos(theta)*((y-ys)*Math.cos(theta)-Math.sin(theta)*dist))));
    
    calcMotorAngVelo = calculatedShootVelocity/(Constants.pivotConstants.DiameterOfWheel/2);
    shootSpeed = calcMotorAngVelo/(Constants.pivotConstants.MaxRPMPivot * Constants.pivotConstants.RPMToRadPS * Constants.pivotConstants.MotorTransferEfficency);
    
  }

  public void shootSpeed(double speed){
    // shootSpeed = speed * Constants.pivotConstants.DistanceToShootSpeedMultiplier;
    double xs = RobotContainer.turretVision.getDistanceToTag() * Constants.pivotConstants.MetersToFeet;
    double ys = Constants.pivotConstants.HeightOfShooter;
    double theta = Math.toRadians(Constants.pivotConstants.AngleOfShooter);
    calculatedShootVelocity = speed * ((4*xs))/(Math.sqrt(-(Math.cos(theta)*((Constants.fieldConstants.HeightOfHub-ys)*Math.cos(theta)-Math.sin(theta)*xs))));
    
    calcMotorAngVelo = calculatedShootVelocity/(Constants.pivotConstants.DiameterOfWheel/2);
    shootSpeed = calcMotorAngVelo/(Constants.pivotConstants.MaxRPMPivot * Constants.pivotConstants.RPMToRadPS * Constants.pivotConstants.MotorTransferEfficency);
    SmartDashboard.putNumber("shootSpeed", shootSpeed);
    SmartDashboard.putNumber("Calculated Shoot Speed", calculatedShootVelocity);
  }
  public void TagTracking(double TagID) {
    

  }

  public void manualMode(boolean manualRotateMode) {
    this.manualRotateMode = manualRotateMode;
  }
}