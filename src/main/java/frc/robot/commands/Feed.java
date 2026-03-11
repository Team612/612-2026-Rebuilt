package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TransferConstants;
import frc.robot.subsystems.Transfer;

public class Feed extends Command {

  private Transfer m_transfer;
  private int timer;
  private BooleanSupplier noPulseSupplier;

  // private int totalTime = TransferConstants.shootTime + TransferConstants.recoveryTime;

  public Feed(Transfer m_transfer, BooleanSupplier noPulseSupplier) {
    this.m_transfer = m_transfer;
    this.noPulseSupplier = noPulseSupplier;
    addRequirements(m_transfer);
    SmartDashboard.putNumber("shootTime", 5);
    SmartDashboard.putNumber("recoveryTime", 22);
  }

  @Override
  public void initialize() {
    // m_transfer.setFeed(TransferConstants.feedSpeed);
    m_transfer.setFeedVoltage(6.6);
  }

  @Override
  public void execute() {
    timer++;
    if ((timer > 50) && (((((timer - SmartDashboard.getNumber("rampUpTime", 50)) % (SmartDashboard.getNumber("shootTime", 5) + SmartDashboard.getNumber("recoveryTime", 22))) < SmartDashboard.getNumber("shootTime", 5)) ) || (noPulseSupplier.getAsBoolean()))){
    // if (m_shooter.safeToShoot() && (((((timer - SmartDashboard.getNumber("rampUpTime", 50)) % (SmartDashboard.getNumber("shootTime", 10) + SmartDashboard.getNumber("recoveryTime", 60))) < SmartDashboard.getNumber("shootTime", 10)) ) || (noPulseButton.getAsBoolean()))){
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
    // m_transfer.setFeed(0);
    m_transfer.setFeedVoltage(0);
    m_transfer.setHopperTop(0);
    m_transfer.setHopperBottom(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
