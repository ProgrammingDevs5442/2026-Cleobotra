// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCommand extends Command {
  XboxController Xbox2 = RobotContainer.xbox2;
  boolean intakeExtended = false;
  boolean pressed = false;
  WaitCommand intakeExtendTimer = new WaitCommand(.5);
  boolean intakeAtPose = false;
  /** Creates a new IntakeCommand. */
  public IntakeCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.xbox2.getLeftBumperButtonPressed()) {
      intakeExtended = false;
      intakeAtPose = false;
      intakeExtendTimer.schedule();
    }
    if (RobotContainer.xbox2.getRightBumperButtonPressed()) {
      intakeExtended = true;
      intakeAtPose = false;
      intakeExtendTimer.schedule();
    }
    // else if (RobotContainer.xbox1.getRightBumperButton() == false) {
    //   pressed = false;
    // }
    if ((intakeAtPose || intakeExtendTimer.isFinished()) && !Robot.isAutonomous) {
      RobotContainer.intake.coastIntake();
    }
    else if (intakeExtended) {
      RobotContainer.intake.extendIntake(Constants.intakeConstants.limit);
    }
    else if (!Robot.isAutonomous){
      RobotContainer.intake.extendIntake(0);
    }


    // if (RobotContainer.xbox2.getPOV() == 270 && !pressed) {
    //   RobotContainer.intake.AdjustIntakeLimit(-1);
    //   pressed = true;
    //   intakeAtPose = false;
    //   intakeExtendTimer.schedule();
    // } else if (RobotContainer.xbox2.getPOV() == 90 && !pressed) {
    //   RobotContainer.intake.AdjustIntakeLimit(1);
    //   pressed = true;
    //   intakeAtPose = false;
    //   intakeExtendTimer.schedule();
    // } else if (RobotContainer.xbox2.getPOV() != 0 && RobotContainer.xbox2.getPOV() != 180 && RobotContainer.xbox2.getPOV() != 90 && RobotContainer.xbox2.getPOV() != 270) {
    //   pressed = false;
    // }
    // else {
    //   RobotContainer.intake.AdjustIntakeLimit(0);
    // }

    if (RobotContainer.intake.inPosition() && !intakeAtPose) {
      intakeAtPose = true;
      intakeExtendTimer.cancel();
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
