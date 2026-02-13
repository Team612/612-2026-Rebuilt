package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstans;

public class Vision extends SubsystemBase {

  public Vision() {}

  private static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  private PhotonCamera frontCamera = new PhotonCamera(VisionConstans.frontCameraName);

  private PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
    aprilTagFieldLayout,
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    VisionConstans.frontCameraTransform
  );

  public PhotonPipelineResult returnLatestCameraResult(){
    return frontCamera.getLatestResult();
  }

  public Optional<EstimatedRobotPose> returnPhotonPos(PhotonPipelineResult result){
    return photonPoseEstimator.update(result);
  }

  @Override
  public void periodic() {}
}
