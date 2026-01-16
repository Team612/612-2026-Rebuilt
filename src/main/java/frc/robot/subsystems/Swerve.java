package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
  
  private TalonFX kraken = new TalonFX(0);

  public Swerve() {}

  @Override
  public void periodic() {
    // kraken.set(0.1);
  }
}
