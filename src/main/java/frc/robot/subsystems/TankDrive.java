package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class TankDrive extends SubsystemBase {

  private final TalonFX leftMotor = new TalonFX(DriveConstants.leftMotorID);
  private final TalonFX rightMotor  =new TalonFX(DriveConstants.rightMotorID);
  private final TalonFX leftMotor2 = new TalonFX(DriveConstants.leftMotor2ID);
  private final TalonFX rightMotor2 = new TalonFX(DriveConstants.rightMotor2ID);

  public TankDrive() {
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    leftMotor.getConfigurator().apply(leftConfig);
    rightMotor.getConfigurator().apply(rightConfig);
 
    leftMotor2.getConfigurator().apply(leftConfig);
    rightMotor2.getConfigurator().apply(rightConfig);
  }

  // drives the robot using a m/s ChassisSpeed
  public void driveRobotRelative(ChassisSpeeds speeds) {
    drive(new ChassisSpeeds(speeds.vxMetersPerSecond*DriveConstants.metersPerSecondToPercent, 0, speeds.omegaRadiansPerSecond*DriveConstants.radiansPerSecondToPercent));
  }

  // drives the robot using a percentage ChassisSpeed
  public void drive(ChassisSpeeds speeds) {
    speeds.omegaRadiansPerSecond = speeds.omegaRadiansPerSecond * DriveConstants.zNecessaryOffset;

    DifferentialDriveWheelSpeeds wheelSpeeds = DriveConstants.driveKinematics.toWheelSpeeds(speeds);

    wheelSpeeds.desaturate(1);

    leftMotor.set(-wheelSpeeds.leftMetersPerSecond);
    rightMotor.set(wheelSpeeds.rightMetersPerSecond);
    leftMotor2.set(-wheelSpeeds.leftMetersPerSecond);
    rightMotor2.set(wheelSpeeds.rightMetersPerSecond);

    SmartDashboard.putNumber("leftPercentCmd", wheelSpeeds.leftMetersPerSecond);
    SmartDashboard.putNumber("rightPercentCmd", wheelSpeeds.rightMetersPerSecond);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("leftSideSpeed", leftMotor.get());
    SmartDashboard.putNumber("rightSideSpeed",rightMotor.get());
  }
}
