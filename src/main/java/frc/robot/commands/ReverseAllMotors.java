package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Transfer;
import frc.robot.Constants.TransferConstants;

public class ReverseAllMotors extends Command {

  private Transfer m_transfer;

  public ReverseAllMotors(Transfer m_transfer) {
    this.m_transfer = m_transfer;
    addRequirements(m_transfer);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_transfer.setHopperTop(-TransferConstants.hopperTopSpeed);
    m_transfer.setHopperBottom(-TransferConstants.hopperBottomSpeed);
    m_transfer.setFeedVoltage(-TransferConstants.feedVolts);
  }

  @Override
  public void end(boolean interrupted) {
    m_transfer.setFeedVoltage(0);
    m_transfer.setHopperTop(0);
    m_transfer.setHopperBottom(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
