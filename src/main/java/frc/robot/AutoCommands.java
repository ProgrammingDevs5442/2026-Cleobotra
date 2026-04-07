// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.net.ContentHandler;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveModes;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoCommands {
  /** Creates a new AutoCommands. */
  
  
  public static Command Intake = new Command() {
    WaitCommand wait = new WaitCommand(1);
    @Override
    public void initialize() {
      wait.schedule();
      RobotContainer.Shooter.setIntakeSpeed(-1);
      DriveModes.driveRobot
            // .withVelocityX(.5 * Constants.driveConstants.MaxSpeed); // Drive forward with negative Y (forward)
            .withRotationalRate(.25);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
      RobotContainer.Shooter.setIntakeSpeed(0);
      // DriveModes.driveRobot.withVelocityX(0);
    }

    @Override
    public boolean isFinished() {
      return wait.isFinished();
    }
  };

  public static Command ExtendIntake = new Command() {
    WaitCommand intakeWait = new WaitCommand(1);
    @Override
    public void initialize() {
      intakeWait.schedule();
      RobotContainer.intake.extendIntake(Constants.intakeConstants.limit);
    }

    @Override
    public void execute() {
   
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
      return intakeWait.isFinished();
    }
  };


  public static Command FluffIntake = new Command() {
    WaitCommand intakeFluffIn = new WaitCommand(0.25);
    WaitCommand intakeFluffOut = new WaitCommand(0.25);
    @Override
    public void initialize() {
      intakeFluffIn.schedule();
      RobotContainer.intake.extendIntake(0);
    }

    @Override 
    public void execute() {
      if (intakeFluffIn.isFinished()) {
        intakeFluffOut.schedule();
        RobotContainer.intake.extendIntake(Constants.intakeConstants.limit);
        intakeFluffIn.cancel();
      }
    }

    @Override
    public boolean isFinished() {
      return intakeFluffOut.isFinished();
    }
  };


  public static Command IntakeOn = new Command() {
    WaitCommand intakeWait = new WaitCommand(.15);
    @Override
    public void initialize() {
      intakeWait.schedule();
      RobotContainer.Shooter.setIntakeSpeed(-1);
    }

    @Override
    public void execute() {
      RobotContainer.Shooter.setIntakeSpeed(-1);
   
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
      return intakeWait.isFinished();
    }
  };

  public static Command IntakeOff = new Command() {
    WaitCommand intakeWait = new WaitCommand(.15);
    @Override
    public void initialize() {
      intakeWait.schedule();
      RobotContainer.Shooter.setIntakeSpeed(0);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
      return intakeWait.isFinished();
    }
  };


  public static Command lineUpShot = new Command() {
    WaitCommand shotDelay = new WaitCommand(2);
    @Override
    public void initialize() {
      shotDelay.schedule();
      RobotContainer.pivort.setAutoTarget(true);
    }

    @Override
    public void execute() {
      RobotContainer.drivetrain.setControl(
        DriveModes.driveRobot
          .withRotationalRate(RobotContainer.pivort.findRotateSpeed(0)) // Drive counterclockwise with negative X (left)
      );
    }

    @Override
    public void end(boolean interrupted) {
       RobotContainer.drivetrain.setControl(
        DriveModes.driveRobot
          .withRotationalRate(0) // Drive counterclockwise with negative X (left)
      );
      RobotContainer.intake.setIntakeSpeed(0);
    }

    @Override
    public boolean isFinished() {
      return shotDelay.isFinished();// || ((Math.abs(RobotContainer.pivort.difference) < 3) && (Math.abs(RobotContainer.pivort.rotateSpeed)) < .5);
    }
  };

  public static Command Shoot = new Command() {
    WaitCommand shotDelay = new WaitCommand(5);
    @Override
    public void initialize() {
      shotDelay.schedule();
      RobotContainer.Shooter.shootAtPosition(
        RobotContainer.isRedAlliance ? Constants.fieldConstants.RedFieldHub : Constants.fieldConstants.BlueFieldHub, 
        .90);
    }

    @Override
    public void execute() {
      RobotContainer.Shooter.feedSpeed(6000);
    }

    @Override
    public void end(boolean interrupted) {
      RobotContainer.Shooter.shootAtPosition(
        RobotContainer.isRedAlliance ? Constants.fieldConstants.RedFieldHub : Constants.fieldConstants.BlueFieldHub, 
        0); 
      RobotContainer.Shooter.feedSpeed(0);
    }

    @Override
    public boolean isFinished() {
      return shotDelay.isFinished();
    }
   };


}
