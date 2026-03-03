package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

  private final SparkFlex upperIntake = new SparkFlex(IntakeConstants.upperIntakeID, MotorType.kBrushless);
  private final SparkFlex lowerIntake = new SparkFlex(IntakeConstants.lowerIntakeID, MotorType.kBrushless);

  public Intake() {
    SparkBaseConfig upperConfig = new SparkFlexConfig();
    SparkBaseConfig lowerConfig = new SparkFlexConfig();

    upperConfig.idleMode(IdleMode.kBrake).inverted(false);
    lowerConfig.idleMode(IdleMode.kBrake).inverted(true);

    upperIntake.configure(upperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    lowerIntake.configure(lowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setIntakeSpeed(double speed) {
    upperIntake.set(speed);
    lowerIntake.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intakeBottomGet",lowerIntake.get());
    SmartDashboard.putNumber("intakeUpperGet",upperIntake.get());
  }
}
