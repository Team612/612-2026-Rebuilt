// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Climb;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;


public class L1 extends Command {

  private CANcoder encoder = new CANcoder(2);
  private double kp = 2;

  private XboxController xboxController = new XboxController(0);
  private PIDController pidController = new PIDController(kp, 0, 0);

  Climb m_climb;
  double rotation;

  public L1(Climb climb) {
    m_climb = climb;
    pidController.enableContinuousInput(-0.5, 0.5 );

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = -xboxController.getRawAxis(1);
    double y = -xboxController.getRawAxis(0);
    double degree = Math.atan2(x,y);
    degree /= (2*Math.PI);
    degree -= 0.25;
    if(degree < -0.5) 
      degree += 1;
    System.out.print(degree);

    //double error = encoder.getAbsolutePosition().getValueAsDouble() - degree;
    m_climb.SetMotor(-pidController.calculate(encoder.getAbsolutePosition().getValueAsDouble(), degree));
    //m_climb.SetMotor(Constants.ClimbConstants.speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climb.SetMotor(0);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //placeholder
    if(m_climb.getMotorPosition() >= Constants.ClimbConstants.height) {
      return true;
    }
    return false;
  }
}
