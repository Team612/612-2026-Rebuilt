package frc.robot.commands;

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
  }

  @Override
  public void initialize() {
    timer = 0;
    m_transfer.setFeedVoltage(TransferConstants.feedVolts);
    m_intake.setUpperIntakeSpeed(IntakeConstants.upperIntakeSpeed);
    m_intake.setLowerIntakeSpeed(IntakeConstants.lowerIntakeSpeed);
    m_intake.setJammerSpeed(IntakeConstants.jammerFeedSpeed);
  }

  @Override
  public void execute() {
    timer++;
    if (timer > TransferConstants.rampUpTime){
      m_transfer.setHopperTop(TransferConstants.hopperTopSpeed);
      m_transfer.setHopperBottom(TransferConstants.hopperBottomSpeed);
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
