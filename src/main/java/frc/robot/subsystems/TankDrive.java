package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TankDrive extends SubsystemBase {
  private final TalonFX leftMotor = new TalonFX(1);
  private final TalonFX rightMotor  =new TalonFX(2);
  private final TalonFX leftMotor2 = new TalonFX(3);
  private final TalonFX rightMotor2 = new TalonFX(4);
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);
  private final Pigeon2 m_gyro;
  private final Pose2d m_pose;
  private final DifferentialDriveOdometry m_odometry;
  private final Alliance m_alliance;
  public TankDrive(Pigeon2 gyro, Pose2d pose) {
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    leftMotor2.setControl(new Follower(1, MotorAlignmentValue.Aligned));
    rightMotor2.setControl(new Follower(2, MotorAlignmentValue.Aligned));
    m_leftEncoder.setDistancePerPulse(Constants.DriveConstants.encoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(Constants.DriveConstants.encoderDistancePerPulse);
    resetEncoders();
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftMotor.getConfigurator().apply(leftConfig);
    rightMotor.getConfigurator().apply(rightConfig);
    m_gyro = gyro;
    m_pose = pose;
    m_alliance = DriverStation.getAlliance().get();
    m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
  }
  public TankDrive(Pigeon2 gyro) {
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    leftMotor2.setControl(new Follower(1, MotorAlignmentValue.Aligned));
    rightMotor2.setControl(new Follower(2, MotorAlignmentValue.Aligned));
    m_leftEncoder.setDistancePerPulse(Constants.DriveConstants.encoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(Constants.DriveConstants.encoderDistancePerPulse);
    resetEncoders();
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftMotor.getConfigurator().apply(leftConfig);
    rightMotor.getConfigurator().apply(rightConfig);
    m_gyro = gyro;
    m_pose = Pose2d.kZero;
    m_alliance = DriverStation.getAlliance().get();
    m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }

  public void drive(double forward, double turn) {
    leftMotor.set(forward + turn);
    rightMotor.set(forward - turn);
  }
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }
  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceMeters() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceMeters() {
    return m_rightEncoder.getDistance();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance() * Constants.DriveConstants.metersToInches;
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance() * Constants.DriveConstants.metersToInches;
  }
  public Pose2d updateOdometry() {
    return m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
  }
  public void resetPosition(Pose2d newPose) {
    m_odometry.resetPose(newPose);
  }
  public void resetToDefaultPose() {
    m_odometry.resetPose(m_pose);
  }
  public Pose2d getPose() {
    if (m_alliance == Alliance.Blue) return new Pose2d(m_odometry.getPoseMeters().getTranslation(), m_gyro.getRotation2d());
    return new Pose2d(m_odometry.getPoseMeters().getTranslation().times(-1.0), m_gyro.getRotation2d());
  }
  public void resetGyro() {
    m_gyro.reset();
  }
  @Override
  public void periodic() {
    updateOdometry();
  }
  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
}