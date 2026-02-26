package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Transfer;
import frc.robot.Constants.TransferConstants;

public class Feed extends Command {

  private final Transfer m_transfer;
  private int timer;

  public Feed(Transfer m_transfer) {
    this.m_transfer = m_transfer;
    addRequirements(m_transfer);
  }

  @Override
  public void initialize() {
    m_transfer.setFeed(TransferConstants.feedSpeed);
    timer = 0;
  }

  @Override
  public void execute() {
    timer++;
    if(timer >= TransferConstants.pauseTime) {
      m_transfer.setHopperTop(TransferConstants.hopperTopSpeed);
      m_transfer.setHopperBottom(TransferConstants.hopperBottomSpeed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_transfer.setFeed(0);
    m_transfer.setHopperTop(0);
    m_transfer.setHopperBottom(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
