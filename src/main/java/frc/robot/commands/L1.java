// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Climb;
import edu.wpi.first.wpilibj2.command.Command;


public class L1 extends Command {
  Climb m_climb;
  double rotation;

  public L1(Climb climb) {
    m_climb = climb;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climb.SetMotor(Constants.ClimbConstants.speed);
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
