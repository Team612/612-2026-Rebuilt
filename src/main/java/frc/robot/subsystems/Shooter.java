package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;

public class Shooter extends SubsystemBase {

  private TalonFX shooterMotor = new TalonFX(ShooterConstants.shooterMotorID);
  private SparkMax tiltMotor = new SparkMax(ShooterConstants.tiltMotorID, MotorType.kBrushed);
  private SparkMax turretMotor = new SparkMax(ShooterConstants.turretMotorID, MotorType.kBrushless);
  
  private static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

  private PhotonCamera shooterCamera = new PhotonCamera(ShooterConstants.shooterCameraName);

  private PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
    aprilTagFieldLayout,
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    new Transform3d() // placeholder, will be updated dynamically
  );

  private PIDController turretPID = new PIDController(ShooterConstants.turretKp, ShooterConstants.turretKi, ShooterConstants.turretKd);
  private PIDController tiltPID = new PIDController(ShooterConstants.tiltKp,ShooterConstants.tiltKi,ShooterConstants.tiltKd);

  public Shooter() {
    turretPID.setIZone(0.1);
    tiltPID.setIZone(0.02); 
  }

  public boolean GetLeftTurretLimits() {
    return turretMotor.getForwardLimitSwitch().isPressed();
  }

  public boolean GetRightTurretLimits() {
    return turretMotor.getReverseLimitSwitch().isPressed();
  }

  public void setShooterMotor(double speed){
    shooterMotor.set(speed);
  }
  public void setTurretMotor(double speed){
    turretMotor.set(speed);
  }

  public void setTiltMotor(double speed){
    tiltMotor.set(speed);
  }
  
  public void setTurretPos(double pos){
    if (pos > ShooterConstants.largestTurretAngle)
      pos = ShooterConstants.largestTurretAngle;
    if (pos < ShooterConstants.smallestTurretAngle)
      pos = ShooterConstants.smallestTurretAngle;
    turretMotor.set(-turretPID.calculate(turretMotor.getEncoder().getPosition() * ShooterConstants.turretEncoderToRadians, pos));
  }
  
  public void setTurretEncoderPos(double encoderPos){
    turretMotor.getEncoder().setPosition(encoderPos);
  }

  public void setTiltPos(double pos){
    tiltMotor.set(-tiltPID.calculate(tiltMotor.getEncoder().getPosition(), pos));
  }
  public void setEncoderTiltPos(double pos){
    tiltMotor.getEncoder().setPosition(pos);
  }

  public double getCurrentTurretAngle() {
    return turretMotor.getEncoder().getPosition() * ShooterConstants.turretEncoderToRadians;
  }

  public double getTurretAngleDriver() {
    return turretMotor.getEncoder().getPosition() * ShooterConstants.turretEncoderToRadians * 180/Math.PI;
  }

  public double getShooterVelocity() {
    return shooterMotor.get();
  }


  public double getRegressionModelTilt(double distance){
    double tilt = 0.05333333 * distance - 1.34066666;
    if (tilt > 0)
      tilt = 0;
    if (tilt < -1.33)
      tilt = -1.33;
    return tilt;
  }

  public double getRegressionModelDutyCycle(double distance){
    return -0.00639338*distance*distance+0.10147*distance+0.401483;
  }

  public boolean hasTag(){
    return shooterCamera.getLatestResult().hasTargets();
  }
  public double[] calculateShootingAnglesWithOfficialOffset() {
    PhotonPipelineResult result = shooterCamera.getLatestResult();
    if (!result.hasTargets()) {
      return new double[]{-1, -1};
    }
    PhotonTrackedTarget target = result.getBestTarget();

    int tagID = target.getFiducialId();
    Transform3d tagToHub = new Transform3d();

    switch (tagID) {
      // blue hub tags counterclockwise order
      case 21:
        tagToHub = new Transform3d(new Translation3d(-0.6034024,0,0.309650003), new Rotation3d(0,0,0));
        break;
      case 24:
        tagToHub = new Transform3d(new Translation3d(-0.6034024,-0.3556,0.309650003), new Rotation3d(0,0,0));
        break;
      case 25:
        tagToHub = new Transform3d(new Translation3d(-0.6034024,0.3556,0.309650003), new Rotation3d(0,0,0));
        break;
      case 26:
        tagToHub = new Transform3d(new Translation3d(-0.6034024,0,0.309650003), new Rotation3d(0,0,0));
        break;
      case 27:
        tagToHub = new Transform3d(new Translation3d(-0.6034024,0.3556,0.309650003), new Rotation3d(0,0,0));
        break;
      case 18:
        tagToHub = new Transform3d(new Translation3d(-0.6034024,0,0.309650003), new Rotation3d(0,0,0));
        break;

      // red hub tags counterclockwise order
      case 5:
        tagToHub = new Transform3d(new Translation3d(-0.6034024,0,0.309650003), new Rotation3d(0,0,0));
        break;
      case 8:
        tagToHub = new Transform3d(new Translation3d(-0.6034024,-0.3556,0.309650003), new Rotation3d(0,0,0));
        break;
      case 9:
        tagToHub = new Transform3d(new Translation3d(-0.6034024,0.3556,0.309650003), new Rotation3d(0,0,0));
        break;
      case 10:
        tagToHub = new Transform3d(new Translation3d(-0.6034024,0,0.309650003), new Rotation3d(0,0,0));
        break;
      case 11:
        tagToHub = new Transform3d(new Translation3d(-0.6034024,0.3556,0.309650003), new Rotation3d(0,0,0));
        break;
      case 2:
        tagToHub = new Transform3d(new Translation3d(-0.6034024,0,0.309650003), new Rotation3d(0,0,0));
        break;

      default:
        return new double[]{-1, -1};
    }

    Transform3d cameraToTag = target.getBestCameraToTarget();

    Transform3d shooterToHub = VisionConstants.shooterToCamera.plus(cameraToTag).plus(tagToHub);

    double angleError = -(Math.atan2(shooterToHub.getX(),shooterToHub.getY())-(Math.PI/2));
    double distance = Math.sqrt(shooterToHub.getX()*shooterToHub.getX()+shooterToHub.getY()*shooterToHub.getY());

    return new double[]{angleError, distance};
  }

  public Transform3d getRobotToCamera() {
    Transform3d turretToCamera = new Transform3d(new Translation3d(ShooterConstants.radiusTurretToCamera, 0, 0).rotateBy(new Rotation3d(0,0,getCurrentTurretAngle())), new Rotation3d(0,0,getCurrentTurretAngle()));
    return ShooterConstants.robotToTurret.plus(turretToCamera);
  }

  public PhotonPipelineResult returnLatestCameraResult() {
    return shooterCamera.getLatestResult();
  }

  // public double distanceToTag() {
  //    if (hasTag()) {
  //     m_shooter.getUn
  //    }
  // }

  public Optional<EstimatedRobotPose> returnPhotonPos(PhotonPipelineResult result) {
    photonPoseEstimator.setRobotToCameraTransform(getRobotToCamera());
    return photonPoseEstimator.update(result);
}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("turretPos",turretMotor.getEncoder().getPosition());
  }
}