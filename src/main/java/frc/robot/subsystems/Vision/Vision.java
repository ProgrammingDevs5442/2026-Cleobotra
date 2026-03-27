// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Telemetry;
import frc.robot.Constants.visionConstants;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix6.Utils;

public class Vision extends SubsystemBase {
  
  public ArrayList<CalculatedCamera> cameras = new ArrayList<CalculatedCamera>();

  
    public final static CalculatedLimelight LimelightCenter = new CalculatedLimelight("limelight-mason");
    public final static CalculatedLimelight LimelightRight = new CalculatedLimelight("limelight-rightii");
    // public final static CalculatedLimelight LimelightLeft = new CalculatedLimelight("limelight-left");


    // PhotonPoseEstimator microsoftPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0)));
    // PhotonPoseEstimator thriftyPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0)));

    Telemetry logger = RobotContainer.logger;

  public Vision() {
    cameras.add(LimelightCenter);
    cameras.add(LimelightRight);
    // cameras.add(LimelightLeft);
  }
  

  public Pose2d getFieldPose() {
    double fX = 0;
    double fY = 0;
    double fR = 0;
    double tot = 0;

    for (CalculatedCamera camera: cameras) {
        if (camera.hasTarget()) {
          fX += camera.getFieldPose().getX() * camera.getTrust();
          fY += camera.getFieldPose().getY() * camera.getTrust();
          fR += camera.getFieldPose().getRotation().getRadians() * camera.getTrust();
          tot += camera.getTrust();
        }
      }
    fX /= tot;
    fY /= tot;
    fR /= tot;
    return new Pose2d(fX,fY, new Rotation2d(fR));
  }
  
  public boolean hasTarget() {
    for (CalculatedCamera camera : cameras) {
      if (camera.hasTarget()) return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    // Update camera readings to be in sync with the robot
    for (CalculatedCamera camera: cameras) {
      camera.updateResult();

      ///// Add to Odometry \\\\\
      if (camera.hasTarget()) {
        double now = Timer.getFPGATimestamp();
        double timestamp = Utils.fpgaToCurrentTime(now);

        // Update drivetrain odometry
        if(!Robot.isAutonomous) {
        RobotContainer.drivetrain.addVisionMeasurement(
          camera.getPathFieldPose(),
          timestamp
        );
        }
      }
    }

    

    SmartDashboard.putNumber("Field pose X", getFieldPose().getX());
    SmartDashboard.putNumber("Field pose Y", getFieldPose().getY());
    SmartDashboard.putNumber("Field pose R", getFieldPose().getRotation().getDegrees());
    
    // SmartDashboard.putNumber("FL Camera X", Limelight2.getTargetPose().getX());
    // SmartDashboard.putNumber("FL Camera Y", Limelight2.getTargetPose().getY());
    // SmartDashboard.putNumber("FL Camera R", Limelight2.getTargetPose().getRotation().getDegrees());
    // SmartDashboard.putNumber("FL Camera Trust", Limelight2.getTrust());
    
  }
}