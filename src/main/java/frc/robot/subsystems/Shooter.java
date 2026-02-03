// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public Shooter() {}
  double calculatedShootVelocity = 0;
  double calcMotorAngVelo = 0;
  double shootSpeed;
  double feedSpeed;
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("dX", RobotContainer.turretVision.TagTracking());
    SmartDashboard.putNumber("dz", RobotContainer.turretVision.getDistanceToTag());
    SendableRegistry.setName(RobotContainer.shootMotorLeft, "Shoot speed");
    

    RobotContainer.shootMotorLeft.set(shootSpeed);
    RobotContainer.shootMotorMiddle.set(shootSpeed);
    RobotContainer.shootMotorRight.set(shootSpeed);
    RobotContainer.feedMotorLeft.set(feedSpeed); 
    RobotContainer.feedMotorMiddle.set(feedSpeed);
    RobotContainer.feedMotorRight.set(feedSpeed);
    RobotContainer.beltMotor.set(-feedSpeed);
  }

  public void shootSpeed(double speed){
    // shootSpeed = speed * Constants.pivotConstants.DistanceToShootSpeedMultiplier;
    double xs = RobotContainer.turretVision.getDistanceToTag() * Constants.measurementConstants.MetersToFeet;
    double ys = Constants.shooterConstants.HeightOfShooter;
    double theta = Math.toRadians(Constants.shooterConstants.AngleOfShooter);
    calculatedShootVelocity = speed * ((4*xs))/(Math.sqrt(-(Math.cos(theta)*((Constants.fieldConstants.HeightOfHub-ys)*Math.cos(theta)-Math.sin(theta)*xs))));
    
    calcMotorAngVelo = calculatedShootVelocity/(Constants.shooterConstants.DiameterOfWheel/2);
    shootSpeed = calcMotorAngVelo/(Constants.pivotConstants.MaxRPMPivot * Constants.measurementConstants.RPMToRadPS * Constants.pivotConstants.MotorTransferEfficency);
    shootSpeed = speed;
    SmartDashboard.putNumber("shootSpeed", shootSpeed);
    SmartDashboard.putNumber("Calculated Shoot Speed", calculatedShootVelocity);
    SmartDashboard.putNumber("Shoot Motor Angular Velocity", RobotContainer.shootMotorLeft.getVelocity().getValueAsDouble());
  }

  public double getVoltage() {
    return RobotContainer.shootMotorLeft.getMotorVoltage().getValueAsDouble();
  }
  
  public void feedSpeed(double speed){
    feedSpeed = speed;
  }
}
