// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.LightID;

@SuppressWarnings("removal")
public class Vision extends SubsystemBase {

  // candle stuff
  private CANdle candle = new CANdle(VisionConstants.CANdleID);

  private final RainbowAnimation rainbow = new RainbowAnimation(1.0, 0.6, 64);

  private final StrobeAnimation red = new StrobeAnimation(255, 0, 0, 0, 0.02, 64);
  private final StrobeAnimation blue = new StrobeAnimation(0, 0, 255, 0, 0.02, 64);

  private boolean police = false;

  private double timer = 0;

  // camera stuff
  private static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  public static final Transform3d frontCameraTransform =
  new Transform3d(
    new edu.wpi.first.math.geometry.Translation3d(
      edu.wpi.first.math.util.Units.inchesToMeters(7.5),
      edu.wpi.first.math.util.Units.inchesToMeters(0),
      edu.wpi.first.math.util.Units.inchesToMeters(8)),
    new edu.wpi.first.math.geometry.Rotation3d()
  );

  private PhotonCamera frontCamera = new PhotonCamera(VisionConstants.frontCameraName);
  private PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
    aprilTagFieldLayout,
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    frontCameraTransform
  );

  public Vision() {}

  public PhotonPipelineResult returnLatestCameraResult(){
    return frontCamera.getLatestResult();
  }

  public Optional<EstimatedRobotPose> returnPhotonPos(PhotonPipelineResult result){
    return photonPoseEstimator.update(result);
  }

  public double getApritTagYaw(){
    PhotonPipelineResult result = frontCamera.getLatestResult();
    if (!result.hasTargets()) 
      return 0.0;
    return result.getBestTarget().getYaw();
  }

  public void setLightID(int id){
    candle.setLEDs(0,0,0);
    candle.clearAnimation(0);

    if (id == LightID.POLICE.id)
      police = true;
    else
      police = false;
    if (id == LightID.RAINBOW.id)
      candle.animate(rainbow);
    if (id == LightID.OFF.id)
      candle.setLEDs(0,0,0);
    if (id == LightID.WHITE.id)
      candle.setLEDs(255,255,255);
    if (id == LightID.RED.id)
      candle.setLEDs(255,0,0);
    if (id == LightID.GREEN.id)
      candle.setLEDs(0,255,0);
    if (id == LightID.BLUE.id)
      candle.setLEDs(0,0,255);
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
  }
}
