package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private static final int LEFT_MOTOR_ID = 10;
  private static final int RIGHT_MOTOR_ID = 11;

  private static final double INTAKE_SPEED = 0.85;
  private static final double OUTTAKE_SPEED = -0.65;

  private final SparkMax leftMotor;
  private final SparkMax rightMotor;

  public Intake() {
    leftMotor = new SparkMax(LEFT_MOTOR_ID, MotorType.kBrushless);
    rightMotor = new SparkMax(RIGHT_MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();

    leftConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .voltageCompensation(12.0)
        .inverted(false);

    rightConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .voltageCompensation(12.0)
        .follow(leftMotor, true);

    leftMotor.configure(
        leftConfig,
        SparkMax.ResetMode.kResetSafeParameters,
        SparkMax.PersistMode.kPersistParameters);

    rightMotor.configure(
        rightConfig,
        SparkMax.ResetMode.kResetSafeParameters,
        SparkMax.PersistMode.kPersistParameters);
  }

  public void intake() {
    leftMotor.set(INTAKE_SPEED);
  }

  public void outtake() {
    leftMotor.set(OUTTAKE_SPEED);
  }

  public void stop() {
    leftMotor.stopMotor();
  }

  @Override
  public void periodic() {
  }
}