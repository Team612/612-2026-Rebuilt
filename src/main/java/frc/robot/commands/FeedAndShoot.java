package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Transfer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TransferConstants;
import frc.robot.subsystems.Shooter;

public class FeedAndShoot extends Command {

  private final Transfer m_transfer;
  private final Shooter m_shooter;
  private int timer;

  public FeedAndShoot(Transfer transfer, Shooter shooter) {
    m_transfer = transfer;
    m_shooter = shooter;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    m_transfer.setFeed(TransferConstants.feedSpeed);
    m_shooter.setShooterVelocity(ShooterConstants.defaultShootSpeed);
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
    m_shooter.setShooterVelocity(0);
    m_transfer.setFeed(0);
    m_transfer.setHopperTop(0);
    m_transfer.setHopperBottom(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
