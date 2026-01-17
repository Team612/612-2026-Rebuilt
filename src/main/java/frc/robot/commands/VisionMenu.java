// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Vision;

public class VisionMenu extends Command {
  private Vision m_vision;
  private CommandXboxController controller;
  private int menuID = 3;

  private boolean lastY = false;
  private boolean lastA = false;

  public VisionMenu(Vision m_vision, CommandXboxController controller) {
    this.m_vision = m_vision;
    this.controller = controller;
    addRequirements(m_vision);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    boolean y = controller.y().getAsBoolean();
    boolean a = controller.a().getAsBoolean();

    if (y && !lastY) {
        menuID++;
    }
    if (a && !lastA) {
        menuID--;
    }

    lastY = y;
    lastA = a;


    if (menuID < 0)
      menuID = 6;
    if (menuID > 6)
      menuID = 0;

    if ((controller.y().getAsBoolean()) || (controller.a().getAsBoolean()))
      m_vision.setLightID(menuID);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
