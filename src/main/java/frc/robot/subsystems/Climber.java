// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.climberConstants;
import frc.robot.RobotContainer;

public class Climber extends SubsystemBase {
  double upAndDownSpeed = 0;
  double upAndDownCurrentPos = climberConstants.PivotToFloorOffset;
  double upAndDownTargetPos = 0;

  double lastHeight = 0;
  double height = 0;
  int intHeight = 0;
  double combinedHeight = 0;

  /** Creates a new Climber. */
  public Climber() {}

  @Override
  public void periodic() {

    height = 0;//RobotContainer.climberEncoder.get();
    if (lastHeight > 0.75 && height < 0.25) intHeight += 1;
    if (lastHeight < 0.25 && height > 0.75) intHeight -= 1;
    combinedHeight = intHeight + height - climberConstants.UpAndDownOffset;
    lastHeight = height;

    upAndDownCurrentPos = getHeight();
    
    RobotContainer.climbMotor.set(upAndDownSpeed);
    // RobotContainer.climbMotor.set(-upAndDownSpeed);
    
    SmartDashboard.putNumber("Elevator Height", getHeight());
    SmartDashboard.putNumber("Elevator Encoder", RobotContainer.climbMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Speed", upAndDownSpeed);
  }

  /**Height of the pivot point in inches from floor*/
  public double getHeight() {
    // return (combinedHeight * climberConstants.InchesPerRotation) + climberConstants.PivotToFloorOffset; // Absolute Encoder, removed 2/20/25
    // return ((RobotContainer.climbMotor.getEncoder().getPosition() * climberConstants.InchesPerRotation) / 25) + climberConstants.PivotToFloorOffset;
    return ((RobotContainer.climbMotor.getPosition().getValueAsDouble() * climberConstants.InchesPerRotation) / 16) + climberConstants.PivotToFloorOffset;
  }

  public void moveUpAndDown (double speed) {
    //Determine speed when in manual mode  
    upAndDownSpeed = speed * climberConstants.UpAndDownSpeedFactor;
    // //if it's past the top limit and still moving up, stop moving and move back down
    // if ((upAndDownSpeed>0) && (upAndDownCurrentPos>=climberConstants.ArmTopLimit)) {
    //   upAndDownSpeed = 0;
    //   upAndDownTargetPos = climberConstants.ArmTopLimit;
    // }

    // //if it's below the bottom limit and still moving down, stop moving and move back up
    // if ((upAndDownSpeed<0) && (upAndDownCurrentPos<=climberConstants.ArmBottomLimit)) {
    //   upAndDownSpeed = 0;
    //   upAndDownTargetPos = climberConstants.ArmBottomLimit;
    // }
  }
}

