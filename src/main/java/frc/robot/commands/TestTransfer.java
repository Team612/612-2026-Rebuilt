package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Transfer;

public class TestTransfer extends Command {
  private final Transfer m_transfer;
  private final CommandXboxController controller;

  public TestTransfer(Transfer m_transfer, CommandXboxController controller) {
    this.m_transfer = m_transfer;
    this.controller = controller;
    addRequirements(m_transfer);
  }

  @Override
  public void initialize() {
    m_transfer.setMotor2(0.5);
    m_transfer.setMotor3(0.5);
  }

  @Override
  public void execute() {
    m_transfer.setMotor1(controller.getLeftY());
    m_transfer.setMotor4(controller.getRightY());
  }

  @Override
  public void end(boolean interrupted) {
    m_transfer.setMotor1(0);
    m_transfer.setMotor2(0);
    m_transfer.setMotor3(0);
    m_transfer.setMotor4(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}