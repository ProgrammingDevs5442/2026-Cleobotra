// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.List;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public Shooter() {}
  double calculatedShootVelocity = 0;
  double calcMotorAngVelo = 0;
  double shootSpeed;
  double feedSpeed;
  double shooterEfficiency = 0.8;
  AngularVelocity kVelocityTolerance = RPM.of(100);
  
  Pose2d pose = RobotContainer.vision.getFieldPose();

  TalonFX leftMotor = RobotContainer.shootMotorLeft;
  TalonFX middleMotor = RobotContainer.shootMotorMiddle;
  TalonFX rightMotor = RobotContainer.shootMotorRight;
  TalonFX fourthMotor = RobotContainer.ExtraShootMotor;
  List<TalonFX> shootMotors = List.of(leftMotor, middleMotor, rightMotor, fourthMotor);
  
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  @Override
  public void periodic() {
    pose = RobotContainer.vision.getFieldPose();
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("dX", RobotContainer.turretVision.TagTracking());
    SmartDashboard.putNumber("dz", RobotContainer.turretVision.getDistanceToTag());
    SendableRegistry.setName(RobotContainer.shootMotorLeft, "Shoot speed");
    SmartDashboard.putNumber("Shoot Motor Speed", RobotContainer.shootMotorLeft.getVelocity().getValueAsDouble());
    
    // double dist = Math.sqrt(Math.pow(x - pose.getX(),2) + Math.pow(z - pose.getY(),2));
    // SmartDashboard.putNumber("Distance to target", dist);
    

    // RobotContainer.shootMotorLeft.set(shootSpeed);
    // RobotContainer.shootMotorMiddle.set(shootSpeed);
    // RobotContainer.shootMotorRight.set(shootSpeed);
    if (shootSpeed != 0) {
      for (final TalonFX motor : shootMotors) {
        motor.setControl(
          velocityRequest
            .withVelocity(RPM.of(shootSpeed))
        );
      }
    }
    else {
      for (final TalonFX motor : shootMotors) {
        motor.set(0);
      }
    }
    // RobotContainer.ExtraShootMotor.setControl(
    //   // voltageRequest.withOutput(Volts.of(feedSpeed * 11.0))
    //   velocityRequest.withVelocity(RPM.of(shootSpeed))// * Constants.shooterConstants.maxRPMFeeder))
    // );
    if (feedSpeed != 0) {    
      RobotContainer.beltMotor.setControl(
        // velocityRequest.withVelocity(RPM.of(feedSpeed))
        new VoltageOut(-feedSpeed/5000 * 11)
      );
      
      RobotContainer.feedMotorLeft.setControl(
        // voltageRequest.withOutput(Volts.of(feedSpeed * 11.0))
        velocityRequest.withVelocity(RPM.of(feedSpeed))// * Constants.shooterConstants.maxRPMFeeder))
      );
    }
    else {
      RobotContainer.beltMotor.set(0);
      RobotContainer.feedMotorLeft.set(0);
      // RobotContainer.ExtraShootMotor.set(0);
    }

  }

  public void shootSpeed(double speed){
    // shootSpeed = speed * Constants.pivotConstants.DistanceToShootSpeedMultiplier;
    double xs = RobotContainer.turretVision.getDistanceToTag() * Constants.measurementConstants.MetersToFeet;
    double ys = Constants.shooterConstants.HeightOfShooter;
    double theta = Math.toRadians(Constants.shooterConstants.AngleOfShooter);
    calculatedShootVelocity = speed * ((4*xs))/(Math.sqrt(-(Math.cos(theta)*((Constants.fieldConstants.HeightOfHub-ys)*Math.cos(theta)-Math.sin(theta)*xs))));
    
    calcMotorAngVelo = calculatedShootVelocity/(Constants.shooterConstants.DiameterOfWheel/2);
    // shootSpeed = calcMotorAngVelo/(Constants.pivotConstants.MaxRPMPivot * Constants.measurementConstants.RPMToRadPS * Constants.pivotConstants.MotorTransferEfficency);
    shootSpeed = speed;
    SmartDashboard.putNumber("shootSpeed", shootSpeed);
    SmartDashboard.putNumber("Calculated Shoot Speed", calculatedShootVelocity);
    SmartDashboard.putNumber("Shoot Motor Angular Velocity", RobotContainer.shootMotorLeft.getVelocity().getValueAsDouble());
  }

  public void shootAtPosition(double x, double y, double z, double speed) {
    x *= Constants.measurementConstants.MetersToFeet;
    y *= Constants.measurementConstants.MetersToFeet;
    z *= Constants.measurementConstants.MetersToFeet;
    //x,y,z is target position; y is vertical
    //speed is a constant factor, 1.15 (might want to change)
    Pose2d pose = RobotContainer.vision.getFieldPose();
    pose = new Pose2d(pose.getX() * Constants.measurementConstants.MetersToFeet, pose.getY() * Constants.measurementConstants.MetersToFeet, pose.getRotation());

    double dist = Math.sqrt(Math.pow(x - pose.getX(),2) + Math.pow(z - pose.getY(),2));
    SmartDashboard.putNumber("Distance to target", dist);
    double ys = Constants.shooterConstants.HeightOfShooter;
    double theta = Math.toRadians(65);
    // double theta = RobotContainer.linearServo.positionToAngle(RobotContainer.linearServo.getPosition());
    // double theta = Math.toRadians(Constants.shooterConstants.AngleOfShooter);
    calculatedShootVelocity = speed * ((4*dist))/(Math.sqrt(-(Math.cos(theta)*((y-ys)*Math.cos(theta)-Math.sin(theta)*dist))));
    
    calcMotorAngVelo = calculatedShootVelocity/(Constants.shooterConstants.DiameterOfWheel/2);
    shootSpeed = shooterEfficiency * speed;//calcMotorAngVelo/(Constants.pivotConstants.MaxRPMPivot * Constants.measurementConstants.RPMToRadPS * shooterEfficiency);
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

  public void modifyEfficiency(double efficiency) {
    shooterEfficiency += efficiency;
    SmartDashboard.putNumber("Shoot Efficiency", shooterEfficiency);
  }
}
