package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transfer;
import frc.robot.Constants.TransferConstants;

public class ReverseAllMotors extends Command {

  private Transfer m_transfer;
  private Shooter m_shooter;

  // private int totalTime = TransferConstants.shootTime + TransferConstants.recoveryTime;

  public ReverseAllMotors(Transfer m_transfer, Shooter m_shooter) {
    this.m_transfer = m_transfer;
    this.m_shooter = m_shooter;
    addRequirements(m_transfer, m_shooter);
  }

  @Override
  public void initialize() {
    
    // m_transfer.setFeedVoltage(6.6);
  }

  @Override
  public void execute() {
    m_transfer.setHopperTop(-TransferConstants.hopperTopSpeed);
    m_transfer.setHopperBottom(-TransferConstants.hopperBottomSpeed);
    m_transfer.setFeed(-TransferConstants.feedSpeed);
    m_shooter.setShooterRPM(-0.6);
  }

  @Override
  public void end(boolean interrupted) {
    m_transfer.setFeed(0);
    // m_transfer.setFeedVoltage(0);
    m_transfer.setHopperTop(0);
    m_transfer.setHopperBottom(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}