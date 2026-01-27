package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import org.photonvision.targeting.PhotonPipelineResult;

public class Swerve extends SubsystemBase {
  
  private SwerveModule frontL;
  private SwerveModule frontR;
  private SwerveModule backL;
  private SwerveModule backR;

  private Rotation2d heading;

  private Pigeon2 gyro;

  private ChassisSpeeds tele = new ChassisSpeeds();
  private ChassisSpeeds auto = new ChassisSpeeds();

  private SwerveModulePosition[] modulePositions;

  private Pose2d initialPose = new Pose2d(0, 0, new Rotation2d());
  private SwerveDrivePoseEstimator poseEstimator;

  private boolean resetTrigger = false;
  private Pose2d resetPos = new Pose2d();

  private Vision m_vision;

  private final Field2d field = new Field2d();

  public Swerve(Vision m_vision) {
    frontL = new SwerveModule(DriveConstants.frontLDriveMotorID, DriveConstants.frontLSteerMotorID, DriveConstants.frontLCANcoderID, DriveConstants.frontLEncoderOffset);
    frontR = new SwerveModule(DriveConstants.frontRDriveMotorID, DriveConstants.frontRSteerMotorID, DriveConstants.frontRCANcoderID, DriveConstants.frontREncoderOffset);
    backL = new SwerveModule(DriveConstants.backLDriveMotorID, DriveConstants.backLSteerMotorID, DriveConstants.backLCANcoderID, DriveConstants.backLEncoderOffset);
    backR = new SwerveModule(DriveConstants.backRDriveMotorID, DriveConstants.backRSteerMotorID, DriveConstants.backRCANcoderID, DriveConstants.backREncoderOffset);

    modulePositions = new SwerveModulePosition[4];
    modulePositions[0] = new SwerveModulePosition();
    modulePositions[1] = new SwerveModulePosition();
    modulePositions[2] = new SwerveModulePosition();
    modulePositions[3] = new SwerveModulePosition();

    poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.swerveKinematics,
      new Rotation2d(),
      getModulePositions(),
      initialPose,
      // Odometry Standard Deviations, x y & z
      VecBuilder.fill(0.003, 0.003, 0.001),
      // Vision measurement std deviations
      VecBuilder.fill(0.025, 0.025, 0.035)
    );

    this.m_vision = m_vision;

    SmartDashboard.putData("Field", field);

    gyro = new Pigeon2(DriveConstants.gyroID);
  }

  private void drive(ChassisSpeeds chassisSpeed){
    SwerveModuleState[] moduleStates = DriveConstants.swerveKinematics.toSwerveModuleStates(chassisSpeed);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, 1);

    frontL.setSwerveState(moduleStates[0]);
    frontR.setSwerveState(moduleStates[1]);
    backL.setSwerveState(moduleStates[2]);
    backR.setSwerveState(moduleStates[3]);
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] tempModulePositions = {frontL.getCurrentWheelPosition(),frontR.getCurrentWheelPosition(),backL.getCurrentWheelPosition(),backR.getCurrentWheelPosition()};
    return tempModulePositions;
  }

  public Pose2d getPose(){
    return poseEstimator.getEstimatedPosition();
  }

  public void setPose(Pose2d p){
    gyro.setYaw(p.getRotation().getDegrees());
    resetTrigger = true;
    resetPos = p;
  }

  public Rotation2d getHeading(){
    return heading;
  }

  public void addVisionMeasurement(Pose2d pos, double timestamp){
    poseEstimator.addVisionMeasurement(pos, timestamp);
  }

  public void setTeleComponent(ChassisSpeeds tele){
    this.tele = tele;
  }
  public void setAutoComponent(ChassisSpeeds auto){
    this.auto = auto;
  }

  @Override
  public void periodic() {
    drive(new ChassisSpeeds(
      tele.vxMetersPerSecond + auto.vxMetersPerSecond,
      tele.vyMetersPerSecond + auto.vyMetersPerSecond,
      tele.omegaRadiansPerSecond + auto.omegaRadiansPerSecond
    ));

    heading = Rotation2d.fromDegrees(Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360));

    poseEstimator.update(
      gyro.getRotation2d(),
      getModulePositions()
    );

    PhotonPipelineResult result = m_vision.returnLatestCameraResult();
    if (result.hasTargets()) {
      if (result.getBestTarget().getPoseAmbiguity() < 0.3){ // test if this is necessary
        var estimatedPoseOptional = m_vision.returnPhotonPos(result);
        if (estimatedPoseOptional.isPresent()) {
          var estimatedPose = estimatedPoseOptional.get().estimatedPose;
          poseEstimator.addVisionMeasurement(estimatedPose.toPose2d(), result.getTimestampSeconds());
        }
      }
    }

    field.setRobotPose(poseEstimator.getEstimatedPosition());

    if (resetTrigger){
      poseEstimator.resetPosition(
        heading,
        getModulePositions(),
        resetPos
      );
      resetTrigger = false;
    }
  }
}
