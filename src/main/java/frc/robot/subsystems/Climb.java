// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

import com.ctre.phoenix6.hardware.TalonFX;

public class Climb extends SubsystemBase {

  private TalonFX motor = new TalonFX(ClimbConstants.climbMotorID);
  public Climb() {
    motor.getPosition().getValueAsDouble();
  }

  public void SetMotor(double value) {
    motor.set(value);
  }
  public double getMotorPosition() {
    return motor.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {}
}
