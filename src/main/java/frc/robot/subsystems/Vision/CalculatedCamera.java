// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
public class CalculatedCamera {

  ////// Values \\\\\
  String key;

  /** Initializes a CalculatedLimelight based on the given camera name. */ 
  public CalculatedCamera(String key) {
    this.key = key;
  }

  /** Returns the name of the camera. */
  public String getKey() {
    return key;
  }

  public void updateResult() {};

  /** Returns true if there is a target in view. */
  public boolean hasTarget() {
    return false;
  }

  /** Returns the ID of the currently visible target, or -1 if none are present. */
  public long getTargetID() {
    return 0;
  }

  /** Returns a value of how accurate we think the camera is at a given moment (higher value = more accurate). */
  public double getTrust() {
    return 0;
  }

  public double getTagAngle() {
    return 0;
  }

  public double getDistanceToTag() {
    return 0;
  }

  /** Returns the field-relative Pose2d. */
  public Pose2d getFieldPose() {
    return new Pose2d();
  }
  
  /** Returns the target-relative Pose2d. */
  public Pose2d getTargetPose() {
      return new Pose2d();
  }
}

