package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private final SparkFlex motor1 = new SparkFlex(9, MotorType.kBrushless);
  private final SparkFlex motor2 = new SparkFlex(10, MotorType.kBrushless);

  @SuppressWarnings("removal")
  public Intake() {
  
  }

  public void setMotor(double speed) {
    motor1.set(-speed);
    motor2.set(-speed);
  }

  @Override
  public void periodic() {
  }
}