package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {

  private TalonFX motor = new TalonFX(ClimbConstants.climbMotorID);

  public Climb() {}

  public void SetMotor(double value) {
    motor.set(value);
  }
 
  public double getMotorPosition() {
    return motor.getPosition().getValueAsDouble();
  }
  public void resetEncoder() {}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("climbPos",motor.getPosition().getValueAsDouble());
  }
}
