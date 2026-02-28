package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.Constants.TransferConstants;

public class FeedAndShoot extends Command {

  private final Transfer m_transfer;
  private final Shooter m_shooter;
  private int timer;

  private int totalTime = TransferConstants.shootTime + TransferConstants.recoveryTime;

  public FeedAndShoot(Transfer m_transfer, Shooter m_shooter) {
    this.m_transfer = m_transfer;
    this.m_shooter = m_shooter;
    addRequirements(m_transfer);
  }

  @Override
  public void initialize() {
    m_shooter.setShooterMotor(0.5);
    m_transfer.setFeed(TransferConstants.feedSpeed);
    timer = 0;
  }

  @Override
  public void execute() {
    timer++;
    if ((timer >= TransferConstants.rampUpTime) && ((timer - TransferConstants.rampUpTime) % totalTime) < TransferConstants.shootTime)  {
      m_transfer.setHopperTop(TransferConstants.hopperTopSpeed);
      m_transfer.setHopperBottom(TransferConstants.hopperBottomSpeed);
    }
    else{
      m_transfer.setHopperTop(0);
      m_transfer.setHopperBottom(0);
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