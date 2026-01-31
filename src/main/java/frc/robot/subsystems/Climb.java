// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;

import com.ctre.phoenix6.hardware.TalonFX;

public class Climb extends SubsystemBase {

  private CANcoder encoder = new CANcoder(2);
  private TalonFX motor = new TalonFX(ClimbConstants.climbMotorID);
  private double kp = 2;
  private XboxController xboxController = new XboxController(0);
  private PIDController pidController = new PIDController(kp, 0, 0);

  public Climb() {
    motor.getPosition().getValueAsDouble();
    pidController.enableContinuousInput(-0.5, 0.5);
    
  }

  public void SetMotor(double value) {
    motor.set(value);
  }
  public double getMotorPosition() {
    return motor.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    double x = -xboxController.getRawAxis(1);
    double y = -xboxController.getRawAxis(0);
    double degree = Math.atan2(x,y);
    degree /= (2*Math.PI);
    degree -= 0.25;
    if(degree < -0.5) 
      degree += 1;
    System.out.print(degree);

    //double error = encoder.getAbsolutePosition().getValueAsDouble() - degree;
    motor.set(-pidController.calculate(encoder.getAbsolutePosition().getValueAsDouble(), degree));
  }
}
