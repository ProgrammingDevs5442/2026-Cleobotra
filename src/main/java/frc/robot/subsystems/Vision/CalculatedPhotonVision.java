package frc.robot.subsystems.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.visionConstants;

public class CalculatedPhotonVision extends CalculatedCamera {

  String camera;
  PhotonCamera cam;
  Transform3d camOffset;


  public CalculatedPhotonVision(String camera, Transform3d camOffset) {
    super(camera);
    this.camera = camera;   
    this.camOffset = camOffset;
    cam = new PhotonCamera(camera);
  }

  public PhotonCamera getPhotonCamera() {
    return cam;
  }

  public PhotonPipelineResult getResult() {
    return cam.getLatestResult();
  }
    
  @Override
    public boolean hasTarget() { 
    return getResult().hasTargets();
  }

  @Override
  public long getTargetID() {
    return getResult().getBestTarget().fiducialId;
  }


  @Override
  public Pose2d getFieldPose() {
    return PhotonUtils.estimateFieldToRobotAprilTag(getResult().getBestTarget().getBestCameraToTarget(),
    AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose((int)getTargetID()).get(),
    camOffset).toPose2d();
    }

  @Override
  public double getTrust() { //TODO might be able to take area the tag takes up and use that instead
    Pose2d targetRelative = getTargetPose();

    return (1 / Math.sqrt((targetRelative.getX() * targetRelative.getX()) + (targetRelative.getY() * targetRelative.getY()) + (visionConstants.AngleDistrust * Math.sin(targetRelative.getRotation().getRadians()))));
  }

  @Override
  public Pose2d getTargetPose() {
    return new Pose2d(
        getResult().getBestTarget().getBestCameraToTarget().getTranslation().toTranslation2d(),
        getResult().getBestTarget().getBestCameraToTarget().getRotation().toRotation2d() // Rotation (pitch)
    );
  }
}
