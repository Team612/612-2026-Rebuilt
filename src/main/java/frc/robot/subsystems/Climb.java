// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
