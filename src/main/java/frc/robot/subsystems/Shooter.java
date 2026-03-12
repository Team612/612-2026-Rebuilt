package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

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

  private SparkMax tiltMotor = new SparkMax(ShooterConstants.tiltMotorID, MotorType.kBrushed);
  private SparkMax turretMotor = new SparkMax(ShooterConstants.turretMotorID, MotorType.kBrushless);
  private TalonFX rightShooterMotor = new TalonFX(ShooterConstants.rightShooterMotorID);
  private TalonFX leftShooterMotor = new TalonFX(ShooterConstants.leftShootermotorID);
  
  private static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  private PhotonCamera shooterCamera = new PhotonCamera(ShooterConstants.shooterCameraName);

  private PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
    aprilTagFieldLayout,
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    new Transform3d() // placeholder, will be updated dynamically
  );

  private PIDController turretPID = new PIDController(ShooterConstants.turretKp, ShooterConstants.turretKi, ShooterConstants.turretKd);
  private PIDController tiltPID = new PIDController(ShooterConstants.tiltKp,ShooterConstants.tiltKi,ShooterConstants.tiltKd);

  private boolean withinShootingRPM = false;

  public Shooter() {
    SparkBaseConfig turretConfig = new SparkMaxConfig();
    SparkBaseConfig tiltConfig = new SparkMaxConfig();

    turretConfig.idleMode(IdleMode.kBrake).inverted(true).limitSwitch.forwardLimitSwitchType(Type.kNormallyClosed).reverseLimitSwitchType(Type.kNormallyOpen);
    tiltConfig.idleMode(IdleMode.kBrake).inverted(false).limitSwitch.forwardLimitSwitchType(Type.kNormallyClosed).reverseLimitSwitchType(Type.kNormallyOpen);

    turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    tiltMotor.configure(tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    turretPID.setIZone(0.1);
    tiltPID.setIZone(0.02);
    turretMotor.getEncoder().setPosition(Math.PI/ShooterConstants.turretEncoderToRadians);

    TalonFXConfiguration rightShooterConfig = new TalonFXConfiguration();
    TalonFXConfiguration leftShooterConfig = new TalonFXConfiguration();

    rightShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightShooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftShooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftShooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    rightShooterMotor.getConfigurator().apply(rightShooterConfig);
    leftShooterMotor.getConfigurator().apply(leftShooterConfig);

    SmartDashboard.putNumber("RPM window", 100);
  }

  public boolean GetForwardTurretLimit() {
    return turretMotor.getForwardLimitSwitch().isPressed();
  }

  public boolean GetReverseTurretLimit() {
    return turretMotor.getReverseLimitSwitch().isPressed();
  }

  public void setShooterRPM(double RPM){
    SmartDashboard.putNumber("RPM", RPM);

    if (RPM == 0){
      leftShooterMotor.setVoltage(0);
      rightShooterMotor.setVoltage(0);
      return;
    }
    double outputVolts = RPM * ShooterConstants.shooterkV + ShooterConstants.shooterkS;
    outputVolts += ShooterConstants.shooterkP * (RPM - rightShooterMotor.getVelocity().getValueAsDouble()*60);
    rightShooterMotor.setVoltage(outputVolts);
    leftShooterMotor.setVoltage(outputVolts);
  }

  public boolean safeToShoot(){
    return withinShootingRPM;
  }

  public void setTurretMotor(double speed){
    turretMotor.set(speed);
  }
  public void setTiltMotor(double speed){
    tiltMotor.set(speed);
  }
  
  public void setTurretPos(double desiredPos){
    if (desiredPos < 0)
      desiredPos += Math.PI;
    else
      desiredPos -= Math.PI;

    if (desiredPos < ShooterConstants.reverseLimit-Math.PI)
      desiredPos = ShooterConstants.reverseLimit-Math.PI;
    if (desiredPos > ShooterConstants.forwardLimit+Math.PI)
      desiredPos = ShooterConstants.forwardLimit+Math.PI;

    double currPos = getCurrentTurretAngle();
    if (currPos < 0)
      currPos += Math.PI;
    else
      currPos -= Math.PI;

    turretMotor.set(turretPID.calculate(currPos, desiredPos));
  }

  public void setTiltPos(double pos){
    tiltMotor.set(-tiltPID.calculate(tiltMotor.getEncoder().getPosition(), pos));
  }
  public void setEncoderTiltPos(double pos){
    tiltMotor.getEncoder().setPosition(pos);
  }

  public void setTurretEncoderPos(double encoderPos){
    turretMotor.getEncoder().setPosition(encoderPos / ShooterConstants.turretEncoderToRadians);
  }
  public double getCurrentTurretAngle() {
    return Math.IEEEremainder(turretMotor.getEncoder().getPosition() * ShooterConstants.turretEncoderToRadians, 2 * Math.PI);
  }

  public double getRegressionModelTilt(double distance){
    double setPosition = 0.169290621824 * distance - 0.297490939512;
    if (setPosition < 0)
      setPosition = 0;
    return setPosition;
  }

  public double getRegressionModelRPM(double distance){
    return -3.40339*distance*distance+318.66126*distance+2219.32905;
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

  public Optional<EstimatedRobotPose> returnPhotonPos(PhotonPipelineResult result) {
    photonPoseEstimator.setRobotToCameraTransform(getRobotToCamera());
    return photonPoseEstimator.update(result);
  }

  @Override
  public void periodic() {
    // System.out.println(getCurrentTurretAngle()+" "+turretMotor.getForwardLimitSwitch().isPressed());
    // System.out.println(getCurrentTurretAngle()+" "+turretMotor.getReverseLimitSwitch().isPressed());

    SmartDashboard.putBoolean("Shooter Has Tag", shooterCamera.getLatestResult().hasTargets());
    SmartDashboard.putNumber("shooterPos",rightShooterMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("shooterVel",rightShooterMotor.getVelocity().getValueAsDouble()*60);
    SmartDashboard.putNumber("rawEncoder",turretMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("turretPosRadians",getCurrentTurretAngle());

    SmartDashboard.putNumber("shooterGet",rightShooterMotor.get());
    SmartDashboard.putNumber("turretGet",turretMotor.get());
    SmartDashboard.putNumber("tiltGet",tiltMotor.get());
    SmartDashboard.putNumber("tiltPos",tiltMotor.getEncoder().getPosition());
  }
}