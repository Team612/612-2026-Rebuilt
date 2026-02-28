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
    // if(m_timer.hasElapsed(0.5)) {
    m_transfer.setFeed(0.6);
      m_shooter.setShooterMotor(-0.5);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_shooter.setShooterMotor(0);
    // m_transfer.setFeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}