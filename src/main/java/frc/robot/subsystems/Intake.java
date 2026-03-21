// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  double intakeSpeed;
  boolean isExtended = false;
  double intakeOffset = 0;
  double targetPose;

  public Intake() {}

    

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
    RobotContainer.intakeMotor.set(intakeSpeed);
    SmartDashboard.putNumber("Intake Speed", intakeSpeed);

    
    SmartDashboard.putNumber("Intake Target Pos", intakeOffset + targetPose);
    SmartDashboard.putNumber("Intake Actual Pos", RobotContainer.intakeExtendMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Intake Offset", intakeOffset);

  }
  public void setIntakeSpeed(double speed){
    intakeSpeed = speed;
  }

  public void extendIntake(double targetPosition){ 
    targetPose = targetPosition;
    
    RobotContainer.intakeExtendMotor.setControl( new PositionDutyCycle(intakeOffset + targetPose));
  }


  public void coastIntake(){
    RobotContainer.intakeExtendMotor.set(0);
  }

  public void AdjustIntakeLimit(double Adjust){
    intakeOffset += Adjust;
  }

  public boolean inPosition(){
    return(Math.abs(intakeOffset + targetPose - RobotContainer.intakeExtendMotor.getPosition().getValueAsDouble()) < 1);
  }
}
