// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Transfer;


public class ManualHopper extends Command {
  private final Transfer m_transfer;
  private final CommandXboxController m_controller;
  private static final double SPEED = 0.2; // yes ik about the repeated instantiations

  public ManualHopper(Transfer transfer, CommandXboxController controller) {
    m_transfer = transfer;
    m_controller = controller;
    addRequirements(m_transfer);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean b = m_controller.getHID().getRawButton(XboxController.Button.kB.value);
    boolean x = m_controller.getHID().getRawButton(XboxController.Button.kX.value);
    boolean y = m_controller.getHID().getRawButton(XboxController.Button.kY.value);
    
    boolean leftBumper = m_controller.getHID().getRawButton(XboxController.Button.kLeftBumper.value);
    boolean rightBumper = m_controller.getHID().getRawButton(XboxController.Button.kRightBumper.value);

    if (rightBumper) {
      m_transfer.setHopperTop(SPEED);
      m_transfer.setHopperBottom(-SPEED);
      m_transfer.setFeed(0.6);
    } else if (leftBumper) {
      m_transfer.setHopperTop(-SPEED);
      m_transfer.setHopperBottom(SPEED);
      m_transfer.setFeed(-0.6);
    } else {
      m_transfer.setHopperTop((y ? SPEED : 0.0));
      m_transfer.setHopperBottom(x ? -SPEED : 0.0);
      m_transfer.setFeed(b ? 0.6 : 0.0);
    }
  }

  // Called once the command ends or is interrupted
  @Override
  public void end(boolean interrupted) {
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}