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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Telemetry;
import frc.robot.Constants.visionConstants;


public class Vision extends SubsystemBase {
  
  public ArrayList<CalculatedCamera> cameras = new ArrayList<CalculatedCamera>();
  public ArrayList<CalculatedCamera> turretCameras = new ArrayList<CalculatedCamera>();

  
    public final static CalculatedLimelight Limelight1 = new CalculatedLimelight("Limelight-left");//visionConstants.cameraOffset);
    public final static CalculatedLimelight Limelight2 = new CalculatedLimelight("limelight-right");//visionConstants.cameraOffset);
    public final static CalculatedLimelight LimelightTurret = new CalculatedLimelight("limelight-turret");//visionConstants.cameraOffset);

    // PhotonPoseEstimator microsoftPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0)));
    // PhotonPoseEstimator thriftyPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(new Translation3d(0,0,0), new Rotation3d(0,0,0)));

    Telemetry logger = RobotContainer.logger;

  public Vision() {
    cameras.add(Limelight1);
    cameras.add(Limelight2);
    turretCameras.add(LimelightTurret);
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

  public boolean hasTarget(ArrayList<CalculatedCamera> List) {
    for (CalculatedCamera camera : List) {
      if (camera.hasTarget()) return true;
    }
    return false;
  }

  // public double angleToTarget() {
  //   double tX = 0;
  //   double tY = 0;
  //   double tot = 0;
  //   for (CalculatedCamera camera: cameras) {
  //     if (hasTarget()) {
  //       tX += camera.getTargetPose().getX() * camera.getTrust();
  //       tY += camera.getTargetPose().getY() * camera.getTrust();
  //       tot += camera.getTrust();
  //     }
  //   }
  //   tX /= tot;
  //   tY = tot;
  //   return Math.atan2(tY, tX);
  // }

  public double TagTracking() {
    double dX = 0;
    for (CalculatedCamera camera: turretCameras) {
      if (hasTarget(turretCameras)) {
        dX = camera.getTagAngle();
      }
    }
    return dX;
  }

  public double getDistanceToTag() {
    double dZ = 0;
    for (CalculatedCamera camera: turretCameras) {
      if (hasTarget(turretCameras)) {
        dZ = camera.getDistanceToTag();
      }
    }
    return dZ;
  };

  @Override
  public void periodic() {
    // Update camera readings to be in sync with the robot
    for (CalculatedCamera camera: cameras) {
      camera.updateResult();
    }

    SmartDashboard.putNumber("FR Camera X", Limelight1.getTargetPose().getX());
    SmartDashboard.putNumber("FR Camera Y", Limelight1.getTargetPose().getY());
    SmartDashboard.putNumber("FR Camera R", Limelight1.getTargetPose().getRotation().getDegrees());
    SmartDashboard.putNumber("FR Camera Trust", Limelight1.getTrust());
    
    SmartDashboard.putNumber("FL Camera X", Limelight2.getTargetPose().getX());
    SmartDashboard.putNumber("FL Camera Y", Limelight2.getTargetPose().getY());
    SmartDashboard.putNumber("FL Camera R", Limelight2.getTargetPose().getRotation().getDegrees());
    SmartDashboard.putNumber("FL Camera Trust", Limelight2.getTrust());
    
  }
}