// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
public class ShooterVision extends SubsystemBase {
  private static AprilTagFieldLayout aprilTagFieldLayout;
  private PhotonPoseEstimator photonPoseEstimator;

  private static ShooterVision visionInstance = null;

  PhotonCamera shooterCamera;
  List<PhotonPipelineResult> shooterCameraResults;
  PhotonPipelineResult shooterLatestResult;



  private CANdle candle = new CANdle(VisionConstants.CANdleID);

  private final RainbowAnimation rainbow = new RainbowAnimation(1.0, 0.6, 64);

  private final StrobeAnimation red = new StrobeAnimation(255, 0, 0, 0, 0.02, 64);
  private final StrobeAnimation blue = new StrobeAnimation(0, 0, 255, 0, 0.02, 64);

  private boolean police = false;

  private double timer = 0;

  public void setLightID(int id){
    candle.setLEDs(0,0,0);
    candle.clearAnimation(0);

    if (id == 1)
      police = true;
    else
      police = false;

    if (id == 0)
      candle.animate(rainbow);
    if (id == 2)
      candle.setLEDs(0,0,0);
    if (id == 3)
      candle.setLEDs(255,255,255);
    if (id == 4)
      candle.setLEDs(255,0,0);
    if (id == 5)
      candle.setLEDs(0,255,0);
    if (id == 6)
      candle.setLEDs(0,0,255);
  }


  private Pose2d robotInTagPose;
  /**
   * Creates a new Vision.
   * 
   * @throws IOException
   **/

  public ShooterVision() {

    shooterCamera = new PhotonCamera(Constants.VisionConstants.shooterCamera); 

    
    shooterCameraResults = shooterCamera.getAllUnreadResults();

    if (shooterCameraResults.isEmpty()){
      shooterLatestResult = new PhotonPipelineResult();
    }
    
    else {
      shooterLatestResult = shooterCameraResults.get(shooterCameraResults.size()-1);
    }

    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark); 
  }

  public static ShooterVision getVisionInstance() {
    if (visionInstance == null) {
      visionInstance = new ShooterVision();
    }
    return visionInstance;
  }


  public PhotonCamera getShooterAprilTagCamera(){
    return shooterCamera; 
  }

  public Transform3d getRobotToShooterCam(){
    return Constants.VisionConstants.shooterCameraTransform;
  }

  public boolean shooterHasCalibration(){
    if (shooterCamera.getDistCoeffs().equals(Optional.empty())){
      return false;
    }
    return true;
  }

  public boolean shooterHasTag(){
    if(shooterCameraResults.isEmpty()){
      return false;
    }
    else{
        if(shooterLatestResult.hasTargets() && shooterLatestResult.getBestTarget().getFiducialId() >= 0){
            return true;
        }
    }
    return false;
  }



  public int frontTagID(){ //returns the ID of the apriltag ift detects, only runs if hasTag and has targets
    if(shooterCameraResults.isEmpty()){
      return -1;
    }
    else{
        if(shooterLatestResult.hasTargets() && shooterLatestResult.getBestTarget().getFiducialId() >= 0){
            return shooterLatestResult.getBestTarget().getFiducialId();
        }
    }
     return -1;
  }


  public PhotonPipelineResult getFrontPipelineResult() {
    if(shooterCameraResults.isEmpty()){
      return null;
    }
    else{
      return shooterLatestResult;
    }
  
  }  

 
  public Pose2d getShooterTagPose() {
    for (PhotonPipelineResult result : shooterCameraResults){
        if (result.hasTargets()){
            PhotonTrackedTarget bestTarget = result.getBestTarget();
            Transform3d tagSpace = bestTarget.getBestCameraToTarget().plus(getRobotToShooterCam());
            return new Pose2d(tagSpace.getX(), tagSpace.getY(), tagSpace.getRotation().toRotation2d()); 
        }
    }
    return new Pose2d();
  }

  public Pose2d getRobotPose(){
    Pose2d tagPose = robotInTagPose;
    return new Pose2d().transformBy(new Transform2d(tagPose.getTranslation(), tagPose.getRotation()));
  }



public double[] calculateShootingAnglesWithOfficialOffset() {
    if (!shooterHasTag()) {
        return new double[]{0.0, 0.0};
    }
    PhotonTrackedTarget target = shooterCamera.getLatestResult().getBestTarget();

    int tagID = target.getFiducialId();

    Transform3d tagPoseCam = target.getBestCameraToTarget();
    Transform3d tagPoseShooter = tagPoseCam.plus(getRobotToShooterCam());
    double tagX = tagPoseShooter.getTranslation().getX();
    double tagY = tagPoseShooter.getTranslation().getY();
    double tagZ = tagPoseShooter.getTranslation().getZ();

    double hubOffsetX = 0.0;
    double hubOffsetY = 0.0;
    double hubOffsetZ = 0.0;

    switch (tagID) {
    case 9:  // Red Hub - Front, lower
        hubOffsetX = 23.77;   // 492.88 - 469.11
        hubOffsetY = -14.0;   // 144.84 - 158.84
        hubOffsetZ = 44.25;
        break;
    case 10: // Red Hub - Front, center
        hubOffsetX = 23.77;   // 492.88 - 469.11
        hubOffsetY = 0.0;     // 158.84 - 158.84
        hubOffsetZ = 44.25;
        break;
    case 25: // Blue Hub - Back, upper
        hubOffsetX = -9.77;   // 158.34 - 168.11
        hubOffsetY = 14.0;    // 172.84 - 158.84
        hubOffsetZ = 44.25;
        break;
    case 26: // Blue Hub - Back, center
        hubOffsetX = -9.77;   // 158.34 - 168.11
        hubOffsetY = 0.0;     // 158.84 - 158.84
        hubOffsetZ = 44.25;
        break;
    default:
        hubOffsetX = 0.0;
        hubOffsetY = 0.0;
        hubOffsetZ = 0.0;
        break;
    }



    double targetX = tagX + hubOffsetX;
    double targetY = tagY + hubOffsetY;
    double targetZ = tagZ + hubOffsetZ;

    double yawRad = Math.atan2(targetX, targetZ);
    double yawDeg = Math.toDegrees(yawRad);

    double horizontalDistance = Math.sqrt(targetX * targetX + targetZ * targetZ);

    double shooterHeight = Constants.ShooterConstants.kShooterHeightMeters;
    double pitchRad = Math.atan2(targetY - shooterHeight, horizontalDistance);
    double pitchDeg = Math.toDegrees(pitchRad);

    SmartDashboard.putNumber("Calculated Turret Yaw", yawDeg);
    SmartDashboard.putNumber("Calculated Shooter Pitch", pitchDeg);

    return new double[]{yawDeg, pitchDeg};
}


  @Override
  public void periodic() {

    if (police){
      timer++;
      if (timer < 25)
        candle.animate(red);
      else
        candle.animate(blue);

      if (timer == 50)
        timer = 0;
    }
    
    
    SmartDashboard.putBoolean("Shooter Camera Has Tag? ", shooterHasTag());
    SmartDashboard.putNumber("Which tag does it have? ", frontTagID());


    Transform3d tagPose = shooterCamera.getLatestResult().getBestTarget().getBestCameraToTarget();
    double tagX = tagPose.getTranslation().getX();
    double tagY = tagPose.getTranslation().getY();
    double tagZ = tagPose.getTranslation().getZ();
    SmartDashboard.putNumber("Tag X", tagX);
    SmartDashboard.putNumber("Tag Y", tagY);
    SmartDashboard.putNumber("Tag Z", tagZ);

    // SmartDashboard.putBoolean("What is the position of the the shooter? ", true);
    // SmartDashboard.putBoolean("What is the offset needed? ", true);
    // SmartDashboard.putBoolean("Distance to Tag? ", true);
    SmartDashboard.putNumber("Yaw:", calculateShootingAnglesWithOfficialOffset()[0]);
    SmartDashboard.putNumber("Pitch:", calculateShootingAnglesWithOfficialOffset()[1]);

  }
}