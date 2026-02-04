// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Climb extends SubsystemBase {

  private SparkMax motor = new SparkMax(ClimbConstants.climbMotorID, MotorType.kBrushless);

  public Climb() {}

  public void SetMotor(double value) {
    motor.set(value);
  }
  
  public double getMotorPosition() {
    return motor.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("climbPos",motor.getEncoder().getPosition());
  }
}
