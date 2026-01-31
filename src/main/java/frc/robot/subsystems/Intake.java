package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private final SparkMax motor = new SparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

  @SuppressWarnings("removal")
  public Intake() {
    SparkMaxConfig motorConfig = new SparkMaxConfig();

    motorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(40)
        .voltageCompensation(12.0)
        .inverted(false);

    motor.configure(
        motorConfig,
        SparkMax.ResetMode.kResetSafeParameters,
        SparkMax.PersistMode.kPersistParameters);
  }

  public void setMotor(double speed) {
    motor.set(speed);
  }

  @Override
  public void periodic() {
  }
}