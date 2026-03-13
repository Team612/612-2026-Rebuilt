package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TransferConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;

public class Feed extends Command {

  private Transfer m_transfer;
  private Intake m_intake;
  private int timer;

  public Feed(Transfer m_transfer, Intake m_intake) {
    this.m_transfer = m_transfer;
    this.m_intake = m_intake;
    addRequirements(m_transfer);
    SmartDashboard.putNumber("Hopper 1",TransferConstants.hopperTopSpeed);
    SmartDashboard.putNumber("Hopper 2",TransferConstants.hopperBottomSpeed);
    SmartDashboard.putNumber("FeedVolts",TransferConstants.feedVolts);
    SmartDashboard.putNumber("AgitatorFeed",IntakeConstants.jammerFeedSpeed);
    SmartDashboard.putNumber("Intake Ball 1",IntakeConstants.upperIntakeSpeed);
    SmartDashboard.putNumber("Intake Ball 2",IntakeConstants.lowerIntakeSpeed);
  }

  @Override
  public void initialize() {
    timer = 0;
    m_transfer.setFeedVoltage(SmartDashboard.getNumber("FeedVolts",  6.6));
    m_intake.setUpperIntakeSpeed(SmartDashboard.getNumber("Intake Ball 1", 0.35));
    m_intake.setLowerIntakeSpeed(SmartDashboard.getNumber("Intake Ball 2",-0.05));
    m_intake.setJammerSpeed(SmartDashboard.getNumber("AgitatorFeed", 0.2));
  }

  @Override
  public void execute() {
    timer++;
    if (timer > TransferConstants.rampUpTime){
      m_transfer.setHopperTop(SmartDashboard.getNumber("Hopper 1",  0.2));
      m_transfer.setHopperBottom(SmartDashboard.getNumber("Hopper 2",  -0.3));
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_transfer.setFeedVoltage(0);
    m_transfer.setHopperTop(0);
    m_transfer.setHopperBottom(0);
    m_intake.setUpperIntakeSpeed(0);
    m_intake.setLowerIntakeSpeed(0);
    m_intake.setJammerSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
