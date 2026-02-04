// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Climb;

public class ArcadeClimb extends Command {

  private Climb m_climb;
  private CommandXboxController xboxController;

  public ArcadeClimb(Climb m_climb, CommandXboxController xboxController) {
    this.m_climb = m_climb;
    this.xboxController = xboxController;
    addRequirements(m_climb);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double y = xboxController.getLeftY();
    m_climb.SetMotor(y);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {

    return false;
  }
}
