// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;



/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  private boolean pressed;
  private double increment = .01;

  private double shootRPM = 5000;
  private double initialRPM = shootRPM + 400;
  private double shootStage = 0; //1 = just started shooting, 0 = shooting

  /** Creates a new ShootCommand. */
  public ShootCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (RobotContainer.xbox2.getAButtonPressed()) shootStage = 1;
    if (RobotContainer.shootMotorLeft.getVelocity().getValueAsDouble() <= shootRPM * RobotContainer.Shooter.shooterEfficiency && shootStage == 2) shootStage = 0;
    if (RobotContainer.xbox2.getAButton()) { //&& Math.abs(RobotContainer.pivort.getDifference()) <= Constants.shooterConstants.ShootDifferenceThreshold) {
      RobotContainer.Shooter.shootAtPosition(4, 6/Constants.measurementConstants.MetersToFeet,4.6, shootStage >= 1 ? initialRPM : shootRPM);
    // } else if (true) {
      // RobotContainer.Shooter.shootSpeed(.7);
    } else {
      RobotContainer.Shooter.shootAtPosition(4, 6/Constants.measurementConstants.MetersToFeet,4.6,0);
    }

    if (RobotContainer.xbox2.getBButton()) {
      RobotContainer.Shooter.feedSpeed(-3000);
      shootStage = 2;
    } else if (RobotContainer.xbox2.getYButton()) {
      // RobotContainer.Shooter.shootSpeed(.7);
      RobotContainer.Shooter.feedSpeed(2000);
    } else {
      RobotContainer.Shooter.feedSpeed(0);
    }
    

    if (RobotContainer.xbox2.getPOV() == 0 && !pressed) {
      RobotContainer.Shooter.modifyEfficiency(increment);
      pressed = true;
    } else if (RobotContainer.xbox2.getPOV() == 180 && !pressed) {
      RobotContainer.Shooter.modifyEfficiency(-increment);
      pressed = true;
    } else if (RobotContainer.xbox2.getPOV() == 90 && !pressed) {
      increment *= 2;
      SmartDashboard.putNumber("Increment", increment);
      pressed = true;
    } else if (RobotContainer.xbox2.getPOV() == 270 && !pressed) {
      increment /= 2;
      SmartDashboard.putNumber("Increment", increment);
      pressed = true;
    } else if (RobotContainer.xbox2.getPOV() != 0 && RobotContainer.xbox2.getPOV() != 180 && RobotContainer.xbox2.getPOV() != 90 && RobotContainer.xbox2.getPOV() != 270) {
      pressed = false;
    }
    else {
      RobotContainer.Shooter.modifyEfficiency(0);
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

  public static class Shot {
        public final double shooterRPM;
        public final double hoodAngle;

        public Shot(double shooterRPM, double hoodAngle) {
            this.shooterRPM = shooterRPM;
            this.hoodAngle = hoodAngle;
        }
    }
}
