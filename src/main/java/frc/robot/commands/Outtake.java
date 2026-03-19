package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transfer;

public class Outtake extends Command {

  private Transfer m_transfer;
  private Intake m_intake;

  public Outtake(Transfer m_transfer, Intake m_intake) {
    this.m_transfer = m_transfer;
    this.m_intake = m_intake;
    addRequirements(m_transfer);
  }

  @Override
  public void initialize() {
    m_intake.setJammerSpeed(-0.2);
    m_intake.setLowerIntakeSpeed(-0.3);
    m_intake.setUpperIntakeSpeed(-0.3);
    m_transfer.setHopperTop(-0.2);
    m_transfer.setHopperBottom(0.3);
    m_transfer.setFeedVoltage(-6.6);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_intake.setJammerSpeed(0);
    m_intake.setLowerIntakeSpeed(0);
    m_intake.setUpperIntakeSpeed(0);
    m_transfer.setHopperTop(0);
    m_transfer.setHopperBottom(0);
    m_transfer.setFeedVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
