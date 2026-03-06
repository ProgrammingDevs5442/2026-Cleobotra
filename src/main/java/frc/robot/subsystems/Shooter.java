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
import frc.robot.ShootCommand.Shot;

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
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Feet;

import edu.wpi.first.units.measure.Distance;


public class Shooter extends SubsystemBase {
  private static final InterpolatingTreeMap<Distance, Shot> distanceToShotMap = new InterpolatingTreeMap<>(
        (startValue, endValue, q) -> 
            InverseInterpolator.forDouble()
                .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
        (startValue, endValue, t) ->
            new Shot(
                Interpolator.forDouble()
                    .interpolate(startValue.shooterRPM, endValue.shooterRPM, t),
                Interpolator.forDouble()
                    .interpolate(startValue.hoodAngle, endValue.hoodAngle, t)
            )
    );

    static {
        distanceToShotMap.put(Feet.of(5.9), new Shot(4000, 75));
        distanceToShotMap.put(Feet.of(10.5), new Shot(4700, 72));
        // distanceToShotMap.put(Feet.of(165.5), new Shot(3650, ));
    }
  
  /** Creates a new Shooter. */
  public Shooter() {}
  double calculatedShootVelocity = 0;
  double calcMotorAngVelo = 0;
  double shootSpeed;
  double feedSpeed;
  public double shooterEfficiency = .8;
  double kVelocityTolerance = 100;
  double intakeSpeed = 0;
  
  Pose2d pose = RobotContainer.vision.getFieldPose();

  TalonFX leftMotor = RobotContainer.shootMotorLeft;//3
  // TalonFX middleMotor = RobotContainer.shootMotorMiddle;//2
  // TalonFX rightMotor = RobotContainer.shootMotorRight;//1
  // TalonFX fourthMotor = RobotContainer.ExtraShootMotor;//4
  // List<TalonFX> shootMotors = List.of(leftMotor, middleMotor, rightMotor, fourthMotor);
  
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
      leftMotor.setControl(velocityRequest.withVelocity(RPM.of(shootSpeed)));
      // middleMotor.setControl(velocityRequest.withVelocity(RPM.of(shootSpeed)));
      // rightMotor.setControl(velocityRequest.withVelocity(RPM.of(shootSpeed)));
      // fourthMotor.setControl(velocityRequest.withVelocity(RPM.of(shootSpeed * .9)));
    }
    else {
      // for (final TalonFX motor : shootMotors) {
      //   motor.set(0);
      // }
      leftMotor.set(0);
    }
    if (feedSpeed != 0) {    
      RobotContainer.beltMotor.setControl(
        // velocityRequest.withVelocity(RPM.of(feedSpeed))
        new VoltageOut(-feedSpeed/5000 * 11)
      );
      
      RobotContainer.feedMotorLeft.set(feedSpeed/6000//Control(
        // voltageRequest.withOutput(Volts.of(feedSpeed * 11.0))
        // velocityRequest.withVelocity(RPM.of(feedSpeed))// * Constants.shooterConstants.maxRPMFeeder))
      );
    }
    else {
      RobotContainer.beltMotor.set(0);
      RobotContainer.feedMotorLeft.set(0);
      // RobotContainer.ExtraShootMotor.set(0);
    }
    RobotContainer.intakeMotor.set(intakeSpeed);

  }

  public void shootSpeed(double speed){
    // shootSpeed = speed * Constants.pivotConstants.DistanceToShootSpeedMultiplier;
    double xs = RobotContainer.turretVision.getDistanceToTag() * Constants.measurementConstants.MetersToFeet;
    double ys = Constants.shooterConstants.HeightOfShooter;
    
    shootSpeed = speed;
    SmartDashboard.putNumber("shootSpeed", shootSpeed);
    SmartDashboard.putNumber("Calculated Shoot Speed", calculatedShootVelocity);
    SmartDashboard.putNumber("Shoot Motor Angular Velocity", RobotContainer.shootMotorLeft.getVelocity().getValueAsDouble());
  }

  public void shootAtPosition(Pose2d targetPose, double speed) {
    double x = targetPose.getX();
    double z = targetPose.getY(); 

    //x,y,z is target position; y is vertical
    //speed is a constant factor, 1.15 (might want to change)
    Pose2d pose = RobotContainer.vision.getFieldPose();
    pose = new Pose2d(pose.getX(), pose.getY(), pose.getRotation());

    Distance dist = Meters.of(Math.sqrt(Math.pow(x - pose.getX(),2) + Math.pow(z - pose.getY(),2)));
    SmartDashboard.putNumber("Distance to target (M)", dist.in(Meters));
    SmartDashboard.putNumber("Distance to target (Ft)", dist.in(Feet));
    final Shot shot = distanceToShotMap.get(dist);
    calculatedShootVelocity = shot.shooterRPM;
    shootSpeed = shooterEfficiency * speed;//calcMotorAngVelo/(Constants.pivotConstants.MaxRPMPivot * Constants.measurementConstants.RPMToRadPS * shooterEfficiency);
    SmartDashboard.putNumber("shootSpeed", shootSpeed);
    SmartDashboard.putNumber("Calculated Shoot Speed", calculatedShootVelocity);
    SmartDashboard.putNumber("Shoot Motor Angular Velocity", RobotContainer.shootMotorLeft.getVelocity().getValueAsDouble());
  }

  public double getVoltage() {
    return RobotContainer.shootMotorLeft.getMotorVoltage().getValueAsDouble();
  }

  public double getVelocity() {
    return RobotContainer.shootMotorLeft.getVelocity().getValueAsDouble();
  }

  public boolean isAtSpeed() {
    return Math.abs(RobotContainer.shootMotorLeft.getVelocity().getValueAsDouble() - shootSpeed) < (shootSpeed * .1); // 10% RPM tolerance
  }
  
  public void feedSpeed(double speed){
    feedSpeed = speed;
  }

  public void modifyEfficiency(double efficiency) {
    shooterEfficiency += efficiency;
    SmartDashboard.putNumber("Shoot Efficiency", shooterEfficiency);
  }

  public void setIntakeSpeed(double intakeSpeed) {
    this.intakeSpeed = intakeSpeed;
  }
}
