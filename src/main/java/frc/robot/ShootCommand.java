// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;




/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootCommand extends Command {
  private boolean pressed;
  private double increment = .01;
  private double shootRPM = 16000;
  private double initialRPM = shootRPM;
  WaitCommand spinUpDelay = new WaitCommand(.25);
  boolean shooting = false;
  boolean revving = false;

  /**1 = spinning up, 0 = shooting, 2 = Shooting at initial speed */
  private double shootStage = 0; 

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

    // if (RobotContainer.xbox2.getAButtonPressed()) shootStage = 1;
    // if (RobotContainer.shootMotorLeft.getVelocity().getValueAsDouble() <= shootRPM * RobotContainer.Shooter.shooterEfficiency && shootStage == 2) shootStage = 0;
    // if (RobotContainer.xbox2.getAButton()) { //&& Math.abs(RobotContainer.pivort.getDifference()) <= Constants.shooterConstants.ShootDifferenceThreshold) {
    //   RobotContainer.Shooter.shootAtPosition(RobotContainer.isRedAlliance ? Constants.fieldConstants.RedFieldHub : Constants.fieldConstants.BlueFieldHub, shootStage >= 1 ? initialRPM : shootRPM);
    // // } else if (true) {
    //   // RobotContainer.Shooter.shootSpeed(.7);
    // } else {
    //   RobotContainer.Shooter.shootAtPosition(RobotContainer.isRedAlliance ? Constants.fieldConstants.RedFieldHub : Constants.fieldConstants.BlueFieldHub, 0);
    // }
    

    if (RobotContainer.xbox1.getRightTriggerAxis() > .5) {
      if (!spinUpDelay.isScheduled() && !shooting) {
        prepareShot();
        spinUpDelay.schedule();//If the timer isn't already running, start it and prepare the shot
      } else if (spinUpDelay.isFinished() || shooting) {
        shooting();
      }
      // shooting(); 
    }
    else if(RobotContainer.xbox1.getRightBumperButton()) {
      reverseFeed();
    }
    else if (!Robot.isAutonomous) {
      endShot();
      if (spinUpDelay.isScheduled()) {
        spinUpDelay.cancel();
      }
    }



    if (RobotContainer.xbox1.getLeftTriggerAxis() > .2) {
      intake();
    }
    //   RobotContainer.Shooter.feedSpeed(-3750);
    //   shootStage = 2;
    // } else if (RobotContainer.xbox2.getYButton()) {
    //   // RobotContainer.Shooter.shootSpeed(.7);
    //   RobotContainer.Shooter.feedSpeed(2000);
    else if (!Robot.isAutonomous){
      stopIntake();
    //   RobotContainer.Shooter.feedSpeed(0);
    }
    


    // if (RobotContainer.xbox2.getPOV() == 0 && !pressed) {
    //   RobotContainer.Shooter.modifyEfficiency(increment);
    //   pressed = true;
    // } else if (RobotContainer.xbox2.getPOV() == 180 && !pressed) {
    //   RobotContainer.Shooter.modifyEfficiency(-increment);
    //   pressed = true;
    // } else if (RobotContainer.xbox2.getPOV() == 90 && !pressed) {
    //   increment *= 2;
    //   SmartDashboard.putNumber("Increment", increment);
    //   pressed = true;
    // } else if (RobotContainer.xbox2.getPOV() == 270 && !pressed) {
    //   increment /= 2;
    //   SmartDashboard.putNumber("Increment", increment);
    //   pressed = true;
    // } else if (RobotContainer.xbox2.getPOV() != 0 && RobotContainer.xbox2.getPOV() != 180 && RobotContainer.xbox2.getPOV() != 90 && RobotContainer.xbox2.getPOV() != 270) {
    //   pressed = false;
    // }
    // else {
    //   RobotContainer.Shooter.modifyEfficiency(0);
    // }
  }

  public void prepareShot() {
    RobotContainer.Shooter.shootAtPosition(
      RobotContainer.isRedAlliance ? Constants.fieldConstants.RedFieldHub : Constants.fieldConstants.BlueFieldHub, 
      1);//initialRPM : shootRPM);
    RobotContainer.Shooter.feedSpeed(-2000);
    // System.out.println("Preparing Shot");
  }

  public void shooting() {
     RobotContainer.Shooter.shootAtPosition(
      RobotContainer.isRedAlliance ? Constants.fieldConstants.RedFieldHub : Constants.fieldConstants.BlueFieldHub, 
      1);//initialRPM : shootRPM); 
    // if(RobotContainer.shootMotorLeft.getVelocity().getValueAsDouble() * 60 >= shootRPM * .3)
     RobotContainer.Shooter.feedSpeed(6000);
    // else RobotContainer.Shooter.feedSpeed(0);
    // System.out.println("Shooting");
    shooting = true;
  }

  public void endShot() {
    RobotContainer.Shooter.shootAtPosition(
      RobotContainer.isRedAlliance ? Constants.fieldConstants.RedFieldHub : Constants.fieldConstants.BlueFieldHub, 
      0); 
    RobotContainer.Shooter.feedSpeed(0);
    // System.out.println("Ending Shot");
    shooting = false;
  }

  public void cycling() {
    RobotContainer.Shooter.shootAtPosition(
      RobotContainer.isRedAlliance ? Constants.fieldConstants.RedFieldHub : Constants.fieldConstants.BlueFieldHub, 
      0.1);
    RobotContainer.Shooter.feedSpeed(6000);
    // System.out.println("Cycling");
  }

  public void reverseFeed() {
    RobotContainer.Shooter.feedSpeed(-2000);
    RobotContainer.Shooter.setIntakeSpeed(-.5);
    // System.out.println("Reversing Feed");
  }

  public void intake() {
    RobotContainer.Shooter.setIntakeSpeed(-1);
    // System.out.println("Intaking");
  }

  public void stopIntake() {
    RobotContainer.Shooter.setIntakeSpeed(0);
    // System.out.println("Stopping Intake");
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
