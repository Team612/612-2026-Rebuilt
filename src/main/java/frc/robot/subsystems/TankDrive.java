package frc.robot.subsystems;

import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class TankDrive extends SubsystemBase {

  private final TalonFX leftMotor = new TalonFX(DriveConstants.leftMotorID);
  private final TalonFX rightMotor  =new TalonFX(DriveConstants.rightMotorID);
  private final TalonFX leftMotor2 = new TalonFX(DriveConstants.leftMotor2ID);
  private final TalonFX rightMotor2 = new TalonFX(DriveConstants.rightMotor2ID);
  private StatusSignal<Per<AngularVelocityUnit, VoltageUnit>> lKV;
  private StatusSignal<Per<AngularVelocityUnit, VoltageUnit>> rKV;



    private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          DriveConstants.kS,
          DriveConstants.kV,
          DriveConstants.kA
      );

  private final DifferentialDrive diffDrive =
      new DifferentialDrive(
          (output) -> leftMotor.set(output),
          (output) -> rightMotor.set(output)
      );
      
  private boolean red;

  private final Pigeon2 gyro = new Pigeon2(DriveConstants.gyroID);

  private DifferentialDrivePoseEstimator poseEstimator;

  private RobotConfig config;

  private final Field2d field = new Field2d();

  private final Vision m_vision;
  private final Shooter m_shooter;

  public TankDrive(Pose2d initialPos, Vision m_vision, Shooter m_shooter) {
    SmartDashboard.putData("Field", field);
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
        red = true;
    }

    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftMotor.getConfigurator().apply(leftConfig);
    rightMotor.getConfigurator().apply(rightConfig);
    lKV = leftMotor.getMotorKV();
    rKV = rightMotor.getMotorKV();
    leftMotor2.setControl(new Follower(DriveConstants.leftMotorID, MotorAlignmentValue.Aligned));
    rightMotor2.setControl(new Follower(DriveConstants.rightMotorID, MotorAlignmentValue.Aligned));

    // this might not be necessary
    leftMotor.setPosition(0);
    rightMotor.setPosition(0);
    gyro.reset();

    poseEstimator = new DifferentialDrivePoseEstimator(
      DriveConstants.driveKinematics,
      getHeading(),
      getLeftDistanceMeters(),
      getRightDistanceMeters(),
      initialPos,
      // Odometry Standard Deviations, x y & z
      VecBuilder.fill(0.006, 0.006, 0.002),
      // Vision measurement std deviations
      VecBuilder.fill(999, 999, 999) // doesn't matter because it gets overwritten
    );

    this.m_vision = m_vision;
    this.m_shooter = m_shooter;

    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }


    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      new PPLTVController(0.00), // PPLTVController is the built in path following controller for differential drive trains
      config, // The robot configuration
      () -> {
      // Boolean supplier that controls when the path will be mirrored for the red alliance
      // This will flip the path being followed to the red side of the field.
      // THE ORIGIN WILL REMAIN ON THE BLUE SIDE


      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
      },
      this // Reference to this subsystem to set requirements
    );
  }

  public double getLeftDistanceMeters(){
    return leftMotor.getPosition().getValueAsDouble() * DriveConstants.encoderToMeters;
  }
  public double getRightDistanceMeters(){
    return rightMotor.getPosition().getValueAsDouble() * DriveConstants.encoderToMeters;
  }
  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360));
  }

  // drives the robot using a m/s ChassisSpeed


    public void driveRobotRelative(ChassisSpeeds speeds) {

      DifferentialDriveWheelSpeeds wheelSpeeds =
          DriveConstants.driveKinematics.toWheelSpeeds(speeds);

      setWheelSpeeds(wheelSpeeds);
    }

    public void setWheelSpeeds(DifferentialDriveWheelSpeeds speeds) {

        speeds.desaturate(DriveConstants.maxAttainableSpeed);

        double leftVelocity = speeds.leftMetersPerSecond;
        double rightVelocity = speeds.rightMetersPerSecond;

        double leftFeedforward = feedforward.calculate(leftVelocity);
        double rightFeedforward = feedforward.calculate(rightVelocity);

        leftMotor.setVoltage(leftFeedforward);
        rightMotor.setVoltage(rightFeedforward);
    }

    public void arcadeDrive(double forward, double rotation) {
        diffDrive.arcadeDrive(forward, rotation);
    }

    public void tankDrive(double left, double right) {
        diffDrive.tankDrive(left, right);
    }


    public Pose2d getPose(){
      return poseEstimator.getEstimatedPosition();
    }

  public void resetPose(Pose2d pos){
    gyro.reset();
    leftMotor.setPosition(0);
    rightMotor.setPosition(0);

    poseEstimator.resetPosition(
      getHeading(),
      getLeftDistanceMeters(),
      getRightDistanceMeters(),
      new Pose2d()
    );
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    double vx = (leftMotor.getVelocity().getValueAsDouble()*DriveConstants.encoderToMeters + rightMotor.getVelocity().getValueAsDouble()*DriveConstants.encoderToMeters) / 2.0;
    double omega = gyro.getAngularVelocityZDevice().getValueAsDouble();

    return new ChassisSpeeds(vx,0,omega);
  }

  @Override
  public void periodic() {
    if (red)
      poseEstimator.update(getHeading().times(-1), -getLeftDistanceMeters(), -getRightDistanceMeters());
    else
      poseEstimator.update(getHeading(), getLeftDistanceMeters(), getRightDistanceMeters());

    PhotonPipelineResult result = m_vision.returnLatestCameraResult();
    if (result.hasTargets()) {
      if (result.getBestTarget().getPoseAmbiguity() < 0.3){ // test if this is necessary
        var estimatedPoseOptional = m_vision.returnPhotonPos(result);
        if (estimatedPoseOptional.isPresent()) {
          var estimatedPose = estimatedPoseOptional.get().estimatedPose;
          poseEstimator.addVisionMeasurement(estimatedPose.toPose2d(), result.getTimestampSeconds(),VecBuilder.fill(0.025, 0.025, 0.035));
        }
      }
    }
    PhotonPipelineResult shooterResult = m_shooter.returnLatestCameraResult();
    if (shooterResult.hasTargets()) {
      if (shooterResult.getBestTarget().getPoseAmbiguity() < 0.3){ // test if this is necessary
        var estimatedPoseOptional = m_shooter.returnPhotonPos(shooterResult);
        if (estimatedPoseOptional.isPresent()) {
          var estimatedPose = estimatedPoseOptional.get().estimatedPose;
          poseEstimator.addVisionMeasurement(estimatedPose.toPose2d(), shooterResult.getTimestampSeconds(),VecBuilder.fill(0.1, 0.1, 0.14));
        }
      }
    }

    field.setRobotPose(poseEstimator.getEstimatedPosition());

    SmartDashboard.putNumber("leftSideSpeed", leftMotor.get());
    SmartDashboard.putNumber("rightSideSpeed",rightMotor.get());

    SmartDashboard.putNumber("gyro velocity", gyro.getAngularVelocityZDevice().getValueAsDouble()); // check if this is in rad/sec or degrees/sec
  }
}
