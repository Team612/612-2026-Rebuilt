package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.PowerDistribution;


import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;

public class Shooter extends SubsystemBase {

  private SparkMax shooterMotor = new SparkMax(ShooterConstants.shooterMotorID, MotorType.kBrushless);
  private SparkMax tiltMotor = new SparkMax(ShooterConstants.tiltMotorID, MotorType.kBrushed);
  private SparkMax turretMotor = new SparkMax(ShooterConstants.turretMotorID, MotorType.kBrushless);
  
  // private SparkClosedLoopController shooterPID;
  // private RelativeEncoder shooterEncoder;

  private PhotonCamera shooterCamera = new PhotonCamera(ShooterConstants.shooterCameraName);

  private PIDController turretPID = new PIDController(ShooterConstants.turretKp, ShooterConstants.turretKi, ShooterConstants.turretKd);
  private PIDController tiltPID = new PIDController(ShooterConstants.tiltKp,ShooterConstants.tiltKi,ShooterConstants.tiltKd);

  private PowerDistribution pdh = new PowerDistribution();

  public Shooter() {
    // shooterEncoder = shooterMotor.getEncoder();
        // shooterPID = shooterMotor.getClosedLoopController();

    //     SparkMaxConfig shooterConfig = new SparkMaxConfig();
    //     shooterConfig.closedLoop
    //         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    //         .p(0.0002)
    //         .i(0)
    //         .d(0)
    //         .velocityFF(0.00017)
    //         .outputRange(-1, 1);
    //     shooterConfig.voltageCompensation(12.0);

    // shooterMotor.configure(shooterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
 
    turretPID.setIZone(0.1);
    tiltPID.setIZone(0.02);
  }

  public boolean GetLeftTurretLimits() {
    return turretMotor.getForwardLimitSwitch().isPressed();
  }


  public double getRPM() {
    return shooterMotor.getEncoder().getVelocity();
  }


  public boolean GetRightTurretLimits() {
    return turretMotor.getReverseLimitSwitch().isPressed();
  }

  // public void setShooterVelocity(double rpm) {
  //     shooterPID.setReference(rpm, ControlType.kVelocity);
  // }

  public void setShooterMotor(double speed){
    shooterMotor.set(speed);
  }

  // public boolean atSpeed(double targetRPM) {
  //     return Math.abs(shooterEncoder.getVelocity() - targetRPM) < 100;
  // }

  public double getRegressionModelRPM(double distance){
    return (-0.00639338*distance*distance+0.10147*distance+0.401483) * 5700;
  }

  public void setTurretMotor(double speed){
    turretMotor.set(speed);
  }
  public void setTiltMotor(double speed){
    tiltMotor.set(speed);
  }
  
  
  public void setTurretPos(double pos){
    SmartDashboard.putNumber("anotherTurretPos",turretMotor.getEncoder().getPosition() * ShooterConstants.turretEncoderToRadians);

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
    double distance = Math.sqrt(shooterToHub.getX()*shooterToHub.getX()+shooterToHub.getY()*shooterToHub.getY()+shooterToHub.getZ()*shooterToHub.getZ());

    return new double[]{angleError, distance};
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("turretForLim",turretMotor.getForwardLimitSwitch().isPressed());
    SmartDashboard.putBoolean("turretRevLim",turretMotor.getReverseLimitSwitch().isPressed());
    SmartDashboard.putBoolean("tiltForLim",tiltMotor.getForwardLimitSwitch().isPressed());
    SmartDashboard.putBoolean("tiltRevLim",tiltMotor.getReverseLimitSwitch().isPressed());

    SmartDashboard.putNumber("turretPos",turretMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("tiltPos",tiltMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Shooter RPM",shooterMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("currentDraw",pdh.getTotalCurrent());

    SmartDashboard.putNumber("turretSpeed",turretMotor.get());
    SmartDashboard.putNumber("shooterSpeed",shooterMotor.get());
    SmartDashboard.putNumber("tiltSpeed",tiltMotor.get());
  }
}