// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Hood extends SubsystemBase {
  /** Creates a new Hood. */
  public Hood() {}

  double angle = 74;
  double angleOffset = 0;
  double position = 0;
  boolean passing = false;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    position = (Math.max(Math.min(angle + angleOffset, 73), 51) * -0.227488) + 17.3412;
    if (passing) {
      RobotContainer.hoodMotor.setControl(new com.ctre.phoenix6.controls.PositionDutyCycle(7.5));
    }
    else {
      RobotContainer.hoodMotor.setControl(new com.ctre.phoenix6.controls.PositionDutyCycle(position));
    }
    SmartDashboard.putNumber("Shoot Motor Position", position);
    SmartDashboard.putNumber("real Shoot Angle", angle + angleOffset);
  }

  public void setPassing(boolean isPassing) {
    passing = isPassing;
  }

  public void setAngle(double angle) {
    this.angle = Math.max(Math.min(angle, 73), 51);
  }
  public void modifyAngle(double angle) {
    angleOffset += angle;
    // if (angle + angleOffset > 73) angleOffset = 73 - angle;
    // if (angle + angleOffset < 51) angleOffset = 51 - angle;
    // angleOffset = Math.floor(angleOffset);
    SmartDashboard.putNumber("Hood modification", angleOffset);
  }

}
