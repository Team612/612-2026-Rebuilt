package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.TransferConstants;

public class ReverseAllMotors extends Command {

  private Transfer m_transfer;
  private Intake m_intake;
  private int timer;

  public ReverseAllMotors(Transfer m_transfer, Intake m_intake) {
    this.m_transfer = m_transfer;
    this.m_intake = m_intake;
    addRequirements(m_transfer);
  }

  @Override
  public void initialize() {
    timer = 0;

    m_intake.setJammerSpeed(IntakeConstants.jammerIntakeSpeed);
  }

  @Override
  public void execute() {
    timer++;
    if (timer > TransferConstants.dumpHopperTime){
      m_transfer.setHopperTop(-TransferConstants.hopperTopSpeed);
      m_transfer.setHopperBottom(-TransferConstants.hopperBottomSpeed);
    }
    if (timer > TransferConstants.dumpFeedTime){
      m_transfer.setFeedVoltage(-TransferConstants.feedVolts);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_transfer.setFeedVoltage(0);
    m_transfer.setHopperTop(0);
    m_transfer.setHopperBottom(0);
    m_intake.setJammerSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
