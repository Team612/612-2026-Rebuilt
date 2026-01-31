package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class TankDrive extends SubsystemBase {
  private final TalonFX leftMotor = new TalonFX(1);
  private final TalonFX rightMotor  =new TalonFX(2);
  private final TalonFX leftMotor2 = new TalonFX(3);
  private final TalonFX rightMotor2 = new TalonFX(4);

  public TankDrive() {
    
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    leftMotor2.setControl(new Follower(1, MotorAlignmentValue.Aligned));
    rightMotor2.setControl(new Follower(2, MotorAlignmentValue.Aligned));

    resetEncoders();
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    leftMotor.getConfigurator().apply(leftConfig);
    rightMotor.getConfigurator().apply(rightConfig);
  }

  public void drive(double forward, double turn) {
    leftMotor.set(forward + turn);
    rightMotor.set(forward - turn);
  }
  public void resetEncoders() {
    leftMotor.setPosition(0);
    rightMotor.setPosition(0);
  }
  public double getLeftDistanceMeters() {
    return leftMotor.getPosition().getValueAsDouble() * DriveConstants.encoderToMeters;
  }

  public double getRightDistanceMeters() {
    return rightMotor.getPosition().getValueAsDouble() * DriveConstants.encoderToMeters;
  }

  @Override
  public void periodic() {
  }
}