package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;

public class Shooter extends SubsystemBase {

  private SparkMax shooterMotor = new SparkMax(ShooterConstants.shooterMotorID, MotorType.kBrushless);
  
  
  private SparkMax tiltMotor = new SparkMax(ShooterConstants.tiltMotorID, MotorType.kBrushed);
  private SparkMaxConfig tiltControllerConfig = new SparkMaxConfig();
  private SparkClosedLoopController tiltController = tiltMotor.getClosedLoopController();

  private SparkMax turretMotor = new SparkMax(ShooterConstants.turretMotorID, MotorType.kBrushless);
  private SparkClosedLoopController turretController = turretMotor.getClosedLoopController();
  private SparkMaxConfig turretControllerConfig = new SparkMaxConfig();
  
  private CANcoder cancoder = new CANcoder(ShooterConstants.cancoderID);
  private PhotonCamera shooterCamera = new PhotonCamera(ShooterConstants.shooterCameraName);

  public Shooter() {
    turretControllerConfig.closedLoop.p(ShooterConstants.turretKp).i(ShooterConstants.turretKi).d(ShooterConstants.turretKd).outputRange(-ShooterConstants.maxTurretSpeed, ShooterConstants.maxTurretSpeed);
    turretMotor.configure(turretControllerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);  

    tiltControllerConfig.closedLoop.p(ShooterConstants.tiltKp).i(ShooterConstants.tiltKi).d(ShooterConstants.tiltKd).outputRange(-0.5, 0.5);
    tiltMotor.configure(tiltControllerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void setShooterMotor(double speed){
    shooterMotor.set(speed);
  }

  public void setTiltMotor(double speed){
    tiltMotor.set(speed);
  }
  public void setTurretMotor(double speed){
    turretMotor.set(speed);
  }
  
  public void setTiltPos(double positionInRad){
    double positionInRotation = positionInRad  /  (2 * Math.PI) * ShooterConstants.tiltGearRatio;
    tiltController.setSetpoint(positionInRotation, ControlType.kPosition);
  }

  public void setTurretPos(double positionInRad){
    double positionInRotation = positionInRad  /  (2 * Math.PI) * ShooterConstants.turretGearRatio;
    turretController.setSetpoint(positionInRotation, ControlType.kPosition);
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

    // switch (tagID) {
    //   case 9:  // Red Hub - Front, lower
    //     hubOffsetX = 23.77;   // 492.88 - 469.11
    //     hubOffsetY = -14.0;   // 144.84 - 158.84
    //     hubOffsetZ = 44.25;
    //     break;
    //   case 10: // Red Hub - Front, center
    //     hubOffsetX = 23.77;   // 492.88 - 469.11
    //     hubOffsetY = 0.0;     // 158.84 - 158.84
    //     hubOffsetZ = 44.25;
    //     break;
    //   case 25: // Blue Hub - Back, upper
    //     hubOffsetX = -9.77;   // 158.34 - 168.11
    //     hubOffsetY = 14.0;    // 172.84 - 158.84
    //     hubOffsetZ = 44.25;
    //     break;
    //   case 26: // Blue Hub - Back, center
    //     hubOffsetX = -9.77;   // 158.34 - 168.11
    //     hubOffsetY = 0.0;     // 158.84 - 158.84
    //     hubOffsetZ = 44.25;
    //     break;
    //   default:
    //     hubOffsetX = 0.0;
    //     hubOffsetY = 0.0;
    //     hubOffsetZ = 0.0;
    //     break;
    // }

    double targetX = tagX + hubOffsetX;
    double targetY = tagY + hubOffsetY;
    double targetZ = tagZ + hubOffsetZ;
    double shooterHeight = ShooterConstants.kShooterHeightMeters;

    double yawRad = Math.atan2(targetX, targetY-shooterHeight);
    double yawDeg = Math.toDegrees(yawRad);

    // double horizontalDistance = Math.sqrt(targetX * targetX + targetZ * targetZ);

    double pitchRad = Math.atan2(targetX, targetZ);
    double pitchDeg = Math.toDegrees(pitchRad);

    SmartDashboard.putNumber("Calculated Turret Yaw", yawDeg);
    SmartDashboard.putNumber("Calculated Shooter Pitch", pitchDeg);

    return new double[]{yawDeg, pitchDeg};
  }

  public double getCurrentTurretAngle() {
    double rotations = turretMotor.getEncoder().getPosition();
    if (rotations < -0.5)
      rotations += 1;
    return rotations * 2 * Math.PI * ShooterConstants.turretGearRatio;
  }

  public double getCurrentTiltAngle() {
    double rotations = tiltMotor.getEncoder().getPosition();
    if (rotations < -0.5)
      rotations += 1;
    return rotations * 2 * Math.PI * ShooterConstants.tiltGearRatio;
  }

  public boolean shooterHasTag() {
    return shooterCamera.getLatestResult().hasTargets();
  }

  public int frontTagID() {
    if (shooterHasTag()) {
      return shooterCamera.getLatestResult().getBestTarget().getFiducialId();
    }
    return -1;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Pos", shooterMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Turret Pos", turretMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Tilt Pos", tiltMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Turret Angle", getCurrentTurretAngle());
    SmartDashboard.putNumber("Tilt Angle", getCurrentTiltAngle());
    SmartDashboard.putNumber("Shooter Velocity", shooterMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter Current", shooterMotor.getOutputCurrent());

    SmartDashboard.putNumber("Shooter Speed", shooterMotor.get());
    SmartDashboard.putNumber("Turret Speed", turretMotor.get());
    SmartDashboard.putNumber("Tilt Speed", tiltMotor.get());


    SmartDashboard.putBoolean("Shooter Camera Has Tag? ", shooterHasTag());
    SmartDashboard.putNumber("Which tag does it have? ", frontTagID());


    

    PhotonPipelineResult result = shooterCamera.getLatestResult();
    if (result.hasTargets()){

      Transform3d cameraToAprilTag = result.getBestTarget().bestCameraToTarget;

      double[] angles = calculateShootingAnglesWithOfficialOffset();
      SmartDashboard.putNumber("April Tag X", cameraToAprilTag.getX());
      SmartDashboard.putNumber("April Tag Y", cameraToAprilTag.getY());
      SmartDashboard.putNumber("April Tag Z", cameraToAprilTag.getZ());
      SmartDashboard.putNumber("Yaw:", angles[0]);
      SmartDashboard.putNumber("Pitch:", angles[1]);

    }
    else{
      SmartDashboard.putNumber("April Tag X", 0);
      SmartDashboard.putNumber("April Tag Y", 0);
      SmartDashboard.putNumber("April Tag Z", 0);
      SmartDashboard.putNumber("Yaw:", 0);
      SmartDashboard.putNumber("Pitch:", 0);

    }
  }
}
