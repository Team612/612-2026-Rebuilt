package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;

public class Shooter extends SubsystemBase {

  private SparkMax shooterMotor = new SparkMax(ShooterConstants.shooterMotorID, MotorType.kBrushless);
  private SparkMax tiltMotor = new SparkMax(ShooterConstants.tiltMotorID, MotorType.kBrushed);
  private SparkMax turretMotor = new SparkMax(ShooterConstants.turretMotorID, MotorType.kBrushless);
  
  private PhotonCamera shooterCamera = new PhotonCamera(ShooterConstants.shooterCameraName);

  private PIDController turretPID = new PIDController(ShooterConstants.turretKp, ShooterConstants.turretKi, ShooterConstants.turretKd);
  private PIDController tiltPID = new PIDController(ShooterConstants.tiltKp,ShooterConstants.tiltKi,ShooterConstants.tiltKd);

  private DigitalInput rightLimit = new DigitalInput(ShooterConstants.rightLimitDIO);
  private DigitalInput leftLimit = new DigitalInput(ShooterConstants.leftLimitDIO);

  public Shooter() {
    turretPID.setIZone(0.1);
    tiltPID.setIZone(0.02);
  }

  public void setShooterMotor(double speed){
    shooterMotor.set(speed);
  }
  public void setTurretMotor(double speed){
    if(leftLimitPressed() && speed<0) {
      speed=0;
    }
    if(rightLimitPressed() && speed>0) {
      speed=0;
    }
    turretMotor.set(speed);
  }
  public void setTiltMotor(double speed){
    tiltMotor.set(speed);
  }
  
  
  public void setTurretPos(double pos){
    SmartDashboard.putNumber("anotherTurretPos",turretMotor.getEncoder().getPosition() * ShooterConstants.turretEncoderToRadians);
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

  public boolean rightLimitPressed() {
    if(!rightLimit.get()) {
      setTurretEncoderPos(ShooterConstants.rightLimit);
    }
    return !rightLimit.get(); 
  }
  public boolean leftLimitPressed() {
    if(!leftLimit.get()) {
      setTurretEncoderPos(ShooterConstants.leftLimit);
    }
    return !leftLimit.get(); 
  }

  public double[] calculateShootingAnglesWithOfficialOffset() {
    PhotonPipelineResult result = shooterCamera.getLatestResult();
    if (!result.hasTargets()) {
      return new double[]{-1, -1};
    }
    PhotonTrackedTarget target = shooterCamera.getLatestResult().getBestTarget();

    int tagID = target.getFiducialId();

    double hubOffsetX = 0.0;
    double hubOffsetY = 0.0;
    double hubOffsetZ = 0.0;

    switch (tagID) {
      case 10:  // Red Hub - center
        hubOffsetX = 0.6034024;
        hubOffsetY = 0.0;
        hubOffsetZ = 0.309650003;
        break;
      case 9: // Red Hub - offset
        hubOffsetX = 0.6034024;
        hubOffsetY = -0.3556;
        hubOffsetZ = 0.309650003;
        break;
      case 26: // Blue Hub - center
        hubOffsetX = 0.6034024;
        hubOffsetY = 0.0;
        hubOffsetZ = 0.309650003;
        break;
      case 25: // Blue Hub - offset
        hubOffsetX = 0.6034024;
        hubOffsetY = -0.3556;
        hubOffsetZ = 0.309650003;
        break;
    }
    if (hubOffsetZ == 0)
      return new double[]{-1, -1};

    Transform3d camToAprilTag = target.getBestCameraToTarget();

    Transform3d shooterToAprilTag = camToAprilTag.plus(VisionConstants.shooterCameraTransform);

    // potentially more elegant way to program it
    // Transform3d shooterToAprilTag = VisionConstants.shooterCameraTransform.plus(camToAprilTag);

    // Transform3d tagToHub =
    // new Transform3d(
    //   new Translation3d(hubOffsetX, hubOffsetY, hubOffsetZ),
    //   new Rotation3d()
    // );

    // Transform3d shooterToHub = shooterToAprilTag.plus(tagToHub);

    // double shooterToHubX = shooterToHub.getX();
    // double shooterToHubY = shooterToHub.getY();
    // double shooterToHubZ = shooterToHub.getZ();

    double shooterToHubX = shooterToAprilTag.getX() + Math.cos(target.getYaw())*hubOffsetX + Math.sin(target.getYaw())*hubOffsetY;
    double shooterToHubY = shooterToAprilTag.getY() + Math.sin(target.getYaw())*hubOffsetX - Math.cos(target.getYaw())*hubOffsetY;
    double shooterToHubZ = shooterToAprilTag.getZ() + hubOffsetZ;

    double angleError = -(Math.atan2(shooterToHubX,shooterToHubY)-(Math.PI/2));

    return new double[]{angleError, Math.sqrt(shooterToHubX*shooterToHubX+shooterToHubY*shooterToHubY+shooterToHubZ*shooterToHubZ)};
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("turretPos",turretMotor.getEncoder().getPosition());
  }
}
