package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;

public class Shooter extends SubsystemBase {

  private SparkMax shooterMotor = new SparkMax(ShooterConstants.shooterMotorID, MotorType.kBrushless);
  private SparkMax turretMotor = new SparkMax(ShooterConstants.turretMotorID, MotorType.kBrushless);
  private SparkMax tiltMotor = new SparkMax(ShooterConstants.tiltMotorID, MotorType.kBrushed);
  
  private CANcoder cancoder = new CANcoder(ShooterConstants.cancoderID);
  private PhotonCamera shooterCamera = new PhotonCamera(ShooterConstants.shooterCameraName);

  private PIDController turretPID = new PIDController(ShooterConstants.turretKp, ShooterConstants.turretKi, ShooterConstants.turretKd);

  public Shooter() {}

  public void setShooterMotor(double speed){
    shooterMotor.set(speed);
  }
  public void setTiltMotor(double speed){
    tiltMotor.set(speed);
  }

  public void setTurretPos(double positionInRad){
    // if ((positionInRad) > ShooterConstants.largestTurretAngle) positionInRad = ShooterConstants.largestTurretAngle;
    // if ((positionInRad) < ShooterConstants.smallestTurretAngle) positionInRad = ShooterConstants.largestTurretAngle;

    turretMotor.set(turretPID.calculate(getCurrentAngle(), positionInRad));
  }

  public double[] calculateShootingAnglesWithOfficialOffset() {
    PhotonPipelineResult result = shooterCamera.getLatestResult();
    if (!result.hasTargets()) {
        return new double[]{0.0, 0.0};
    }
    PhotonTrackedTarget target = shooterCamera.getLatestResult().getBestTarget();

    int tagID = target.getFiducialId();

    Transform3d tagPoseCam = target.getBestCameraToTarget();
    Transform3d tagPoseShooter = tagPoseCam.plus(VisionConstants.shooterCameraTransform);
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

    double shooterHeight = ShooterConstants.kShooterHeightMeters;
    double pitchRad = Math.atan2(targetY - shooterHeight, horizontalDistance);
    double pitchDeg = Math.toDegrees(pitchRad);

    SmartDashboard.putNumber("Calculated Turret Yaw", yawDeg);
    SmartDashboard.putNumber("Calculated Shooter Pitch", pitchDeg);

    return new double[]{yawDeg, pitchDeg};
  }

  public double getCurrentAngle() {
    double rotations = cancoder.getAbsolutePosition().getValueAsDouble() - ShooterConstants.turretCANcoderOffset;
    if (rotations < -0.5)
      rotations += 1;
    return rotations * 2 * Math.PI;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Pos", shooterMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Turret Pos", turretMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Tilt Pos", tiltMotor.getEncoder().getPosition());

    SmartDashboard.putNumber("Shooter Speed", shooterMotor.get());
    SmartDashboard.putNumber("Turret Speed", turretMotor.get());
    SmartDashboard.putNumber("Tilt Speed", tiltMotor.get());

    PhotonPipelineResult result = shooterCamera.getLatestResult();
    if (result.hasTargets()){

      Transform3d cameraToAprilTag = result.getBestTarget().bestCameraToTarget;

      SmartDashboard.putNumber("April Tag X", cameraToAprilTag.getX());
      SmartDashboard.putNumber("April Tag Y", cameraToAprilTag.getY());
      SmartDashboard.putNumber("April Tag Z", cameraToAprilTag.getZ());
    }
    else{
      SmartDashboard.putNumber("April Tag X", 0);
      SmartDashboard.putNumber("April Tag Y", 0);
      SmartDashboard.putNumber("April Tag Z", 0);
    }
  }
}
