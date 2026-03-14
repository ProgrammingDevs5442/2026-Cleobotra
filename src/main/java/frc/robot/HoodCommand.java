// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoodCommand extends Command {
  /** Creates a new HoodCommand. */
  public HoodCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.hood);
  }

  boolean pressed = false;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.xbox1.getPOV() == 270 && !pressed) {
      RobotContainer.hood.modifyAngle(2);
      pressed = true;
    } else if (RobotContainer.xbox1.getPOV() == 90 && !pressed) {
      RobotContainer.hood.modifyAngle(-2);
      pressed = true;
    } else if (RobotContainer.xbox1.getPOV() != 0 && RobotContainer.xbox1.getPOV() != 180 && RobotContainer.xbox1.getPOV() != 90 && RobotContainer.xbox1.getPOV() != 270) {
      pressed = false;
    }
    else {
      RobotContainer.hood.modifyAngle(0);
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
