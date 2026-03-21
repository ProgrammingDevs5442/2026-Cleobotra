// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.ShootCommand.Shot;

import com.ctre.phoenix6.hardware.TalonFX;
import java.util.List;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;

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
        //This is where you can include calibration points
        //If you want it to actually interpolate instead of just choosing the nearest point then make your data type just a double instead of a custom data type
        distanceToShotMap.put(Feet.of(14), new Shot(6000, 72));
        distanceToShotMap.put(Feet.of(11.2), new Shot(5500, 55));
        distanceToShotMap.put(Feet.of(7.5), new Shot(5200, 75));
        distanceToShotMap.put(Meters.of(2), new Shot(5300, 75));
    }
  
  /** Creates a new Shooter. */
  public Shooter() {}
  double calculatedShootVelocity = 0;
  double calcMotorAngVelo = 0;
  double shootSpeed;
  double feedSpeed;
  public double shooterEfficiency = 1;
  double intakeSpeed = 0;
  
  Pose2d pose = RobotContainer.vision.getFieldPose();

  TalonFX leftMotor = RobotContainer.shootMotorLeft;//3
  TalonFX middleMotor = RobotContainer.shootMotorMiddle;//2
  TalonFX rightMotor = RobotContainer.shootMotorRight;//1
  TalonFX fourthMotor = RobotContainer.ExtraShootMotor;//4
  List<TalonFX> shootMotors = List.of( middleMotor, rightMotor, fourthMotor, leftMotor);
  
  private final VelocityDutyCycle velocityRequest = new VelocityDutyCycle(0).withSlot(0).withEnableFOC(false);

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    pose = RobotContainer.vision.getFieldPose();

    SendableRegistry.setName(RobotContainer.shootMotorLeft, "Shoot speed");
    SmartDashboard.putNumber("Real Shoot Motor Speed", getVelocity());
    
    
    // double dist = Math.sqrt(Math.pow(x - pose.getX(),2) + Math.pow(z - pose.getY(),2));
    // SmartDashboard.putNumber("Distance to target", dist);
    
    // Set speed of shoot motors to a specific velocity and maintain that. To actually use this you need to have pid values, duty cycle doesn't
    if (shootSpeed != 0) {
      for (final TalonFX motor : shootMotors) {
        motor.setControl(velocityRequest.withVelocity(RPM.of(shootSpeed)));
      }
    }
    else { //This is here so that when we don't need the motors running, they coast out instead of agressively stopping
      for (final TalonFX motor : shootMotors) motor.set(0);
    }

    // Set speed of feed and belt motors, since its a duty cycle you don't need pid values
    if (feedSpeed != 0) {    
      RobotContainer.beltMotor.setControl(new DutyCycleOut(feedSpeed * 0.7));
      RobotContainer.feedMotorLeft.setControl(new DutyCycleOut(feedSpeed));
    }
    else {
      RobotContainer.beltMotor.set(0);
      RobotContainer.feedMotorLeft.set(0);
    }

    // Set speed of intake
    RobotContainer.intakeMotor.set(intakeSpeed);

  }

  public void setShootSpeed(double speed){
    shootSpeed = speed;
  }

  //Takes in the pose of the hub it is shooting at and whether you are shooting
  public void shootAtPosition(Pose2d targetPose, double speed) {
    double x = targetPose.getX();
    double y = targetPose.getY(); 
    // x,y,z is target position; z is vertical(depending on what coord system you use it might be different)

    //The pose of the robot on the field
    Pose2d pose = RobotContainer.vision.getFieldPose();
    pose = new Pose2d(pose.getX(), pose.getY(), pose.getRotation());

    //Distance formula from robot pose on field to pose of target on field in meters
    Distance dist = Meters.of(Math.sqrt(Math.pow(x - pose.getX(),2) + Math.pow(y - pose.getY(),2)));
    final Shot shot = distanceToShotMap.get(dist);
    calculatedShootVelocity = shot.shooterRPM;
    
    shootSpeed = (calculatedShootVelocity) * speed;

    SmartDashboard.putNumber("Calced Hood Angle", shot.hoodAngle);
    SmartDashboard.putNumber("Distance to target (M)", dist.in(Meters));
    SmartDashboard.putNumber("Distance to target (Ft)", dist.in(Feet));
    SmartDashboard.putNumber("shootSpeed", shootSpeed);
    SmartDashboard.putNumber("Calculated Shoot Speed", calculatedShootVelocity);
    
    RobotContainer.hood.setAngle(distanceToShotMap.get(dist).hoodAngle);
  }

  public double getVelocity() {
    return RobotContainer.shootMotorLeft.getVelocity().getValueAsDouble();
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
