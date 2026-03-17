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
  double position = 0;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    position = (angle * -0.2382) + 17.555;
    RobotContainer.hoodMotor.setControl(new com.ctre.phoenix6.controls.PositionDutyCycle(position));
    
    SmartDashboard.putNumber("Shoot Motor Position", position);
    SmartDashboard.putNumber("Shoot Angle", angle);
  }

  public void modifyAngle(double angle) {
    this.angle = Math.max(Math.min(this.angle + angle, 73), 49);
  }

}
