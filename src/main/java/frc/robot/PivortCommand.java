// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.pivotConstants;

import java.lang.Math;
import java.math.RoundingMode;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivortCommand extends Command {
  /** Creates a new PivortCommand. */
  public PivortCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.pivort);
  }

  boolean targeting = false;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Send the value from the angle of the controller in radians, coverted to degrees
    if (Math.sqrt(Math.pow(RobotContainer.xbox2.getLeftY(),2) + Math.pow(RobotContainer.xbox2.getLeftX(),2)) > pivotConstants.ShooterDeadzone) {
      RobotContainer.pivort.manualMode(true);
      RobotContainer.pivort.setTargetAngle(Math.toDegrees(Math.atan2(RobotContainer.xbox2.getLeftY(), RobotContainer.xbox2.getLeftX())));
    // } else if (RobotContainer.turretVision.hasTarget(RobotContainer.turretVision.turretCameras)) {
    } 
    else {
      RobotContainer.pivort.manualMode(false);
      // if (targeting) { //TODO TEST THIS
      //   RobotContainer.pivort.shootAtPosition(0, Constants.fieldConstants.HeightOfHub, 0, Constants.pivotConstants.ShootSpeed);;
      // } else {
      //   RobotContainer.pivort.setAutoRotate(0);
      // }
      if (RobotContainer.turretVision.hasTarget(RobotContainer.turretVision.turretCameras)) {
        RobotContainer.pivort.setAutoRotate(RobotContainer.turretVision.TagTracking() + 6);
      } else {
        RobotContainer.pivort.setAutoRotate(0);
      }
    }

    
    if (RobotContainer.xbox2.getRightBumperButton()) {
      targeting = true;
    }


    if (RobotContainer.xbox2.getAButton()) {
      RobotContainer.pivort.shootSpeed(Constants.pivotConstants.ShootSpeed);
    } else {
      RobotContainer.pivort.shootSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
