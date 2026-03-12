package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TransferConstants;
import frc.robot.subsystems.Transfer;

public class Feed extends Command {

  private Transfer m_transfer;
  private int timer;

  public Feed(Transfer m_transfer) {
    this.m_transfer = m_transfer;
    addRequirements(m_transfer);
    SmartDashboard.putNumber("Hopper 1",  0.5);
    SmartDashboard.putNumber("Hopper 2", -0.5);
    SmartDashboard.putNumber("Feed", 6.6);
  }

  @Override
  public void initialize() {
    timer = 0;
    m_transfer.setFeedVoltage(SmartDashboard.getNumber("Feed", 6.6));
  }

  @Override
  public void execute() {
    timer++;
    if (timer > TransferConstants.rampUpTime){
      m_transfer.setHopperTop(SmartDashboard.getNumber("Hopper 1",  0.5));
      m_transfer.setHopperBottom(SmartDashboard.getNumber("Hopper 2",  -0.5));
    }
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
