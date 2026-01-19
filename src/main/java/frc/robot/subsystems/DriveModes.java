// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import frc.robot.RobotContainer;
import frc.robot.Constants.driveConstants;

import com.ctre.phoenix6.swerve.SwerveRequest;

/** Add your docs here. */
public class DriveModes {
    public static final SwerveRequest.FieldCentric driveField = new SwerveRequest.FieldCentric()
            .withDeadband(driveConstants.MaxSpeed * 0.1)
            .withRotationalDeadband(driveConstants.MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
            .withVelocityX(-RobotContainer.joystick.getLeftY() * driveConstants.MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-RobotContainer.joystick.getLeftX() * driveConstants.MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-RobotContainer.joystick.getRightX() * driveConstants.MaxAngularRate); // Drive counterclockwise with negative X (left)
    
    public static final SwerveRequest.RobotCentric driveRobot = new SwerveRequest.RobotCentric()
            .withDeadband(driveConstants.MaxSpeed * 0.1)
            .withRotationalDeadband(driveConstants.MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
            .withVelocityX(-RobotContainer.joystick.getLeftY() * driveConstants.MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-RobotContainer.joystick.getLeftX() * driveConstants.MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-RobotContainer.joystick.getRightX() * driveConstants.MaxAngularRate); // Drive counterclockwise with negative X (left)
    
    public static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    
}